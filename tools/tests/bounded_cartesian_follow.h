#pragma once
// Experimental offline producer only. Not linked into any firmware target.
// At most five external IK calls/update; fixed storage, no allocation or RTOS.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include "stream_protocol.h"

namespace BoundedFollow {
constexpr double Dt=.020;
constexpr double Lower[7]={-175,-75,0,-270,-100,-180,-250};
constexpr double Upper[7]={175,90,180,270,100,180,250};
constexpr double Origin[7]={0,-75,180,0,0,0,0};
constexpr double Factor[7]={50.*51200/360,-50.*51200/360,-50.*51200/360,
    -50.*51200/360,-50.*51200/360,-30.*51200/360,-51200./5};
inline double MaxV(unsigned j){return j==6?20:40;}
inline double MaxA(unsigned j){return j==6?40:80;}
inline bool Finite(double value){uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));return (bits&UINT64_C(0x7ff0000000000000))!=UINT64_C(0x7ff0000000000000);}
inline double Clamp(double x,double lo,double hi){return x<lo?lo:x>hi?hi:x;}
struct Vec {
    double x=0,y=0,z=0;
    Vec operator+(Vec b)const{return {x+b.x,y+b.y,z+b.z};}
    Vec operator-(Vec b)const{return {x-b.x,y-b.y,z-b.z};}
    Vec operator*(double s)const{return {x*s,y*s,z*s};}
};
inline double Dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline double Norm(Vec a){return std::sqrt(Dot(a,a));}
struct State {double q[7]{},v[7]{},a[7]{};};
inline double Margin(const State& s){
    double value=1;
    for(unsigned j=0;j<6;++j)value=std::fmin(value,std::fmin(s.q[j]-Lower[j],Upper[j]-s.q[j])/(Upper[j]-Lower[j]));
    return value;
}
inline State Decode(const StreamProtocol::Record& r){
    State s;
    for(unsigned j=0;j<7;++j){s.q[j]=Origin[j]+r.axes[j].q/Factor[j];
        s.v[j]=r.axes[j].v/Factor[j];s.a[j]=r.axes[j].a/Factor[j];}
    return s;
}
inline bool Encode(const State& s,uint16_t sequence,StreamProtocol::Record& r){
    r={};r.session=1;r.sequence=sequence;r.flags=sequence==0?StreamProtocol::SEED:0;
    for(unsigned j=0;j<7;++j){
        const double values[3]={(s.q[j]-Origin[j])*Factor[j],s.v[j]*Factor[j],s.a[j]*Factor[j]};
        int32_t integers[3];
        for(unsigned k=0;k<3;++k){if(!Finite(values[k])||std::fabs(values[k])>2147483000.)return false;
            integers[k]=int32_t(std::round(values[k]));}
        r.axes[j]={integers[0],integers[1],integers[2]};
    }
    return true;
}

// Bernstein controls make full-horizon bounds sufficient and bounded in work.
struct Polynomial {
    double b[7][6]{},duration=Dt;
    void Build(const State& from,const State& to,double seconds){
        duration=seconds;
        for(unsigned j=0;j<7;++j){double* p=b[j];
            p[0]=from.q[j];p[1]=from.q[j]+from.v[j]*seconds/5;
            p[2]=from.q[j]+2*from.v[j]*seconds/5+from.a[j]*seconds*seconds/20;
            p[5]=to.q[j];p[4]=to.q[j]-to.v[j]*seconds/5;
            p[3]=to.q[j]-2*to.v[j]*seconds/5+to.a[j]*seconds*seconds/20;
        }
    }
    static double Value(const double* p,unsigned count,double u){
        double work[6]{};for(unsigned j=0;j<count;++j)work[j]=p[j];
        for(unsigned n=count-1;n>0;--n)for(unsigned j=0;j<n;++j)work[j]+=u*(work[j+1]-work[j]);
        return work[0];
    }
    State At(double time)const{
        State s;double u=Clamp(time/duration,0,1);
        for(unsigned j=0;j<7;++j){double v[5],a[4];
            for(unsigned k=0;k<5;++k)v[k]=5*(b[j][k+1]-b[j][k])/duration;
            for(unsigned k=0;k<4;++k)a[k]=4*(v[k+1]-v[k])/duration;
            s.q[j]=Value(b[j],6,u);s.v[j]=Value(v,5,u);s.a[j]=Value(a,4,u);
        }
        return s;
    }
    bool Bounded(double ratio)const{
        if(!(duration>=Dt&&duration<=6))return false;
        for(unsigned j=0;j<7;++j){
            for(unsigned k=0;k<6;++k)if(!Finite(b[j][k])||b[j][k]<Lower[j]+.0005||b[j][k]>Upper[j]-.0005)return false;
            for(unsigned k=0;k<5;++k)if(std::fabs(5*(b[j][k+1]-b[j][k])/duration)>MaxV(j)*ratio)return false;
            for(unsigned k=0;k<4;++k)if(std::fabs(20*(b[j][k+2]-2*b[j][k+1]+b[j][k])/(duration*duration))>MaxA(j)*ratio)return false;
        }
        return true;
    }
};

struct Input {
    Vec target{},velocity{};
    uint32_t generation=0;
    bool joystick=false,automaticRail=false;
};
struct Observation {
    unsigned ik=0,curveTrials=0,brakeTrials=0,railCandidates=0;
    bool requested=false,planned=false,acceptedInput=false,fallback=false,braking=false,fatal=false;
    bool railSearch=false,railChanged=false;
    unsigned reason=0; //0 ok,1 IK,2 horizon bounds,3 wire/geometry,4 budget,5 brake failure.
    double horizon=0,railTarget=0,armMargin=0;
};
class Port {
public:
    virtual bool Inverse(Vec tcp,double rail,const State& seed,State& goal)=0;
    // Includes encoded entire-segment bounds and production motor admission.
    virtual bool Segment(const StreamProtocol::Record& from,const StreamProtocol::Record& to)=0;
    virtual ~Port()=default;
};

class Producer {
public:
    enum Variant { DIRECT20_UNCHECKED, BOUNDED_CHORD, BOUNDED_TANGENT, BOUNDED_CHORD_ENVELOPE };
    Producer(Port& io,const State& initial,Vec start,Variant kind):port(io),variant(kind),intent(start){
        fatal=!Encode(initial,0,last);railTarget=initial.q[6];
        for(unsigned j=0;j<7;++j)fatal=fatal||initial.q[j]<Lower[j]||initial.q[j]>Upper[j]||initial.v[j]!=0||initial.a[j]!=0;
    }
    const StreamProtocol::Record& Last()const{return last;}
    Vec Intent()const{return intent;}
    double RailTarget()const{return railTarget;}
    bool Idle()const{return !active;}
    bool Failed()const{return fatal;}
    // Bound includes rail searches, failed solves and tangent estimation.
    Observation Update(const Input& input,StreamProtocol::Record& emitted){
        Observation o;State from=Decode(last);o.armMargin=Margin(from);o.railTarget=railTarget;
        if(fatal){o.fatal=true;return o;}
        const Vec test=input.joystick?input.velocity:input.target;
        const bool validInput=Finite(test.x)&&Finite(test.y)&&Finite(test.z);
        const bool movingJoystick=validInput&&input.joystick&&Norm(input.velocity)>1e-6;
        if(validInput){if(input.joystick)intent=intent+input.velocity*Dt;else intent=input.target;}
        else{o.reason=6;o.fallback=true;}
        const bool newGeneration=!haveInput||input.generation!=acceptedGeneration;
        const bool railDue=input.automaticRail&&(o.armMargin<.10||recovering)&&updates%5==0;
        const bool needPlan=validInput&&(movingJoystick||newGeneration||railDue);
        o.requested=needPlan||!validInput;
        if(needPlan){
            Vec goal=intent+(movingJoystick?input.velocity*.120:Vec{});
            State solved{};bool found=false;
            if(input.automaticRail&&(o.armMargin<.10||recovering)){
                recovering=o.armMargin<.25;
                o.railSearch=recovering&&(updates%5==0);
            }
            double candidates[5]={railTarget,Clamp(from.q[6],-249,249),
                Clamp(from.q[6]+5,-249,249),Clamp(from.q[6]-5,-249,249),Clamp(from.q[6]+15,-249,249)};
            // Include both directions on search frames; global joint margin scores all6 axes.
            if(o.railSearch){candidates[1]=Clamp(from.q[6]+5,-249,249);candidates[2]=Clamp(from.q[6]-5,-249,249);
                candidates[3]=Clamp(from.q[6]+15,-249,249);candidates[4]=Clamp(from.q[6]-15,-249,249);}
            double best=-1e100;unsigned maxCandidates=o.railSearch?4:1;
            for(unsigned n=0;n<maxCandidates&&o.ik<5;++n){
                State test{};if(!Solve(goal,candidates[n],from,test,o)){
                    if(n==0&&input.automaticRail&&!o.railSearch){maxCandidates=5;o.railSearch=true;}
                    continue;
                }
                const double score=Margin(test)-.0001*std::fabs(candidates[n]-from.q[6]);
                if(!found||score>best+.002){solved=test;best=score;found=true;}
            }
            if(found){
                if(movingJoystick){
                    for(unsigned j=0;j<6;++j)solved.v[j]=(solved.q[j]-from.q[j])/.120;
                    if(variant==BOUNDED_TANGENT&&o.ik<5){
                        State tangent{};Vec nextGoal=goal+input.velocity*Dt;
                        if(Solve(nextGoal,solved.q[6],solved,tangent,o))
                            for(unsigned j=0;j<6;++j)solved.v[j]=(tangent.q[j]-solved.q[j])/Dt;
                    }
                    // Rail ends at rest. Arm tangent must compensate its current movement.
                    solved.v[6]=0;
                }
                if(variant==DIRECT20_UNCHECKED){
                    for(unsigned j=0;j<7;++j)solved.v[j]=solved.a[j]=0;
                    StreamProtocol::Record candidate;
                    if(!Encode(solved,last.sequence+1,candidate)){fatal=true;o.fatal=true;return o;}
                    last=candidate;emitted=last;o.planned=o.acceptedInput=true;acceptedGeneration=input.generation;haveInput=true;
                    ++updates;return o;
                }
                double seconds=.120;
                for(unsigned j=0;j<7;++j){
                    const double velocityCoefficient=variant==BOUNDED_CHORD_ENVELOPE?5:2;
                    seconds=std::fmax(seconds,velocityCoefficient*std::fabs(solved.q[j]-from.q[j])/(MaxV(j)*.8));
                    const double coefficient=variant==BOUNDED_CHORD_ENVELOPE?20:6;
                    seconds=std::fmax(seconds,std::sqrt(coefficient*std::fabs(solved.q[j]-from.q[j])/(MaxA(j)*.8)));
                }
                bool planned=false;
                for(unsigned attempt=0;attempt<5;++attempt){
                    const double cap=variant==BOUNDED_CHORD_ENVELOPE?5:3;
                    seconds=std::ceil(seconds/Dt)*Dt;if(seconds>cap)break;
                    Polynomial path,tail;path.Build(from,solved,seconds);++o.curveTrials;
                    if(path.Bounded(.8)&&Brake(solved,tail,&o.brakeTrials)){
                        StreamProtocol::Record first;
                        if(Encode(path.At(Dt),last.sequence+1,first)&&port.Segment(last,first)){
                            tracking=path;brake=tail;at=0;active=true;inBrake=false;planned=true;
                            o.planned=o.acceptedInput=true;o.horizon=seconds;o.reason=0;
                            o.railChanged=std::fabs(railTarget-solved.q[6])>.001;railTarget=solved.q[6];
                            acceptedGeneration=input.generation;haveInput=true;break;
                        }
                        o.reason=3;
                    }else o.reason=2;
                    seconds*=1.45;
                }
                if(!planned)o.fallback=true;
            }else{o.reason=o.ik==5?4:1;o.fallback=true;}
        }
        if(!active){
            State stop=from;
            bool still=true;for(unsigned j=0;j<7;++j)still=still&&std::fabs(from.v[j])<1e-10&&std::fabs(from.a[j])<1e-10;
            if(!still){
                if(!Brake(from,brake,&o.brakeTrials)){fatal=true;o.fatal=true;o.reason=5;return o;}
                active=true;inBrake=true;at=0;o.braking=true;
            }else{
                if(!Encode(stop,last.sequence+1,emitted)||!port.Segment(last,emitted)){fatal=true;o.fatal=true;o.reason=3;return o;}
                last=emitted;++updates;return o;
            }
        }
        Polynomial* current=inBrake?&brake:&tracking;
        bool emittingBrake=inBrake;
        double next=std::fmin(at+Dt,current->duration);
        State to=current->At(next);
        // Preserve exact endpoint rest rather than tiny derivative cancellation noise.
        if(next>=current->duration-1e-8){
            if(inBrake)for(unsigned j=0;j<7;++j)to.v[j]=to.a[j]=0;
        }
        StreamProtocol::Record candidate;
        if(!Encode(to,last.sequence+1,candidate)||!port.Segment(last,candidate)){
            // Zero-IK bounded brake from the actual previous integer endpoint.
            if(!Brake(from,brake,&o.brakeTrials)){fatal=true;o.fatal=true;o.reason=5;return o;}
            inBrake=true;emittingBrake=true;at=0;active=true;o.fallback=o.braking=true;
            to=brake.At(Dt);
            if(!Encode(to,last.sequence+1,candidate)||!port.Segment(last,candidate)){fatal=true;o.fatal=true;o.reason=3;return o;}
            next=Dt;current=&brake;
        }
        last=candidate;emitted=last;at=next;
        if(at>=current->duration-1e-8){
            if(inBrake)active=false;
            else{at=0;inBrake=true;}
        }
        o.braking=o.braking||emittingBrake;o.railTarget=railTarget;++updates;return o;
    }
private:
    Port& port;Variant variant;StreamProtocol::Record last{};
    Polynomial tracking{},brake{};
    Vec intent{};double at=0,railTarget=0;
    uint32_t acceptedGeneration=0,updates=0;
    bool haveInput=false,active=false,inBrake=false,recovering=false,fatal=false;
    bool Solve(Vec tcp,double rail,const State& seed,State& out,Observation& o){
        if(o.ik>=5){o.reason=4;return false;}
        ++o.ik;++o.railCandidates;return port.Inverse(tcp,rail,seed,out);
    }
    static bool Brake(const State& from,Polynomial& polynomial,unsigned* trials=nullptr){
        double seconds=.120;
        for(unsigned j=0;j<7;++j)seconds=std::fmax(seconds,3*std::fabs(from.v[j])/MaxA(j));
        for(unsigned n=0;n<6;++n){
            if(trials)++*trials;
            seconds=std::ceil(seconds/Dt)*Dt;if(seconds>3)return false;
            State stop{};
            for(unsigned j=0;j<7;++j)stop.q[j]=from.q[j]+.5*from.v[j]*seconds+from.a[j]*seconds*seconds/12;
            polynomial.Build(from,stop,seconds);
            if(polynomial.Bounded(.98))return true;
            seconds*=1.4;
        }
        return false;
    }
};
} //namespace BoundedFollow
