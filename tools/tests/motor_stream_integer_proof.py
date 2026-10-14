"""Independent exact quintic, compressed-map and int64 bounds for Curve.Build.

The polynomial uses arbitrary precision integers/Fraction. No ARM throughput
claim is made. The generated per-tick fixture checks the actual C++ Curve.
"""
from fractions import Fraction as F
from math import comb, lcm
from pathlib import Path
import argparse
import hashlib
import json
import random
import re
import struct

SCALE=1<<40
LIMITS=[40000,1536000,1536000,5120000,5120000]
INT64_MAX=(1<<63)-1

def polynomial(f,t):
    """Normalized-time Hermite quintic times 50000, q0-relative."""
    d=t[0]-f[0];v0,v1=f[1],t[1];a0,a1=f[2],t[2]
    return [0,1000*v0,10*a0,
            500000*d-6000*v0-4000*v1-30*a0+10*a1,
            -750000*d+8000*v0+7000*v1+30*a0-20*a1,
            300000*d-3000*v0-3000*v1-10*a0+10*a1]

def exact_point(f,t,tick):
    c=polynomial(f,t)
    return (F(sum(c[k]*tick**k*400**(5-k) for k in range(6)),50000*400**5),
            F(sum(k*c[k]*tick**(k-1)*400**(5-k) for k in range(1,6)),1000*400**4))

def nearest(value):
    q,r=divmod(abs(value.numerator),value.denominator)
    q+=2*r>=value.denominator
    return -q if value<0 else q

def maps():
    rows=[]
    for family,count in [('p',5),('v',4)]:
        for order in range(1,count+1):
            coefficients=[]
            for column in range(5):
                f=[0,0,0];t=[0,0,0]
                if column==0:t[0]=1
                elif column==1:f[1]=1
                elif column==2:t[1]=1
                elif column==3:f[2]=1
                else:t[2]=1
                coefficients.append(sum((-1)**(order-j)*comb(order,j)*
                                        exact_point(f,t,j)[family=='v']
                                        for j in range(order+1))*SCALE)
            rows.append(dict(family=family,order=order,coefficients=coefficients))
    return rows

def accepted(f,t,vl=1536000,al=5120000,home=0):
    if not 0<vl<=1536000 or not 0<al<=5120000:return False
    d=t[0]-f[0]
    b=[0,200*f[1],400*f[1]+f[2],50000*d-400*t[1]+t[2],50000*d-200*t[1],50000*d]
    for i,x in enumerate(b):
        q=f[0]*50000+x
        if not -(1<<31)*50000<=q<=((1<<31)-1)*50000:return False
        if not -(1<<31)*50000<=q+home*50000<=((1<<31)-1)*50000:return False
        if abs(x)>40000*50000:return False
        if i<5 and abs(b[i+1]-x)>vl*200:return False
        if i<4 and abs(b[i+2]-2*b[i+1]+x)>al:return False
    return True

def prove(source):
    text=source.read_text(encoding='utf8');rows=maps()
    body=re.search(r'Coefficient map\[9\]\[5\] = \{(.*?)\n    \};',text,re.S)
    assert body
    actual=[tuple(map(int,x)) for x in re.findall(r'\{(\d+)U,(\d+)U\}',body[1])]
    assert len(actual)==45
    signs=list(map(int,re.search(r'negative\[9\] = \{([^}]+)\}',text)[1].split(',')))
    assert len(signs)==9
    bound_rows=[];max_whole=max_fractional=0
    for ri,row in enumerate(rows):
        values=row['coefficients'];den=lcm(*(x.denominator for x in values))
        assert den==[6103515625,1220703125,244140625,1220703125,1220703125,
                     1953125,1953125,1953125,1953125][ri]
        whole=[int(x) for x in values]
        remainder=[int((x-w)*den) for x,w in zip(values,whole)]
        for ci,(w,r) in enumerate(zip(whole,remainder)):
            assert actual[5*ri+ci]==(abs(w)&0xffffffff,abs(r))
            assert bool(signs[ri]&(1<<ci))==(w<0 or r<0)
            assert abs(r)<=0xffffffff
            assert abs(w)>>32==((2 if ri==5 else 4) if ci==0 and ri in (5,6) else 0)
        whole_bound=sum(abs(w)*v for w,v in zip(whole,LIMITS))
        fraction_bound=sum(abs(r)*v for r,v in zip(remainder,LIMITS))
        assert whole_bound+fraction_bound//den+1<INT64_MAX
        assert fraction_bound+den//2<INT64_MAX
        max_whole=max(max_whole,whole_bound);max_fractional=max(max_fractional,fraction_bound)
        bound_rows.append(dict(family=row['family'],order=row['order'],denominator=den,
            whole_sum_bound=whole_bound,remainder_sum_bound=fraction_bound))
    # Rounded initial differences are within 1/2 Q40 unit of exact ones.
    # Binomial transport is exact for 400 increasing forward-difference ticks.
    p=[F(0)]+[sum(abs(c)*v for c,v in zip(r['coefficients'],LIMITS))+F(1,2) for r in rows[:5]]
    v=[F(1536000*SCALE)]+[sum(abs(c)*n for c,n in zip(r['coefficients'],LIMITS))+F(1,2) for r in rows[5:]]
    pb=[sum(comb(400,k-i)*p[k] for k in range(i,6)) for i in range(6)]
    vb=[sum(comb(400,k-i)*v[k] for k in range(i,5)) for i in range(5)]
    error_q=F(sum(comb(400,k) for k in range(1,6)),2*SCALE)
    error_v=F(sum(comb(400,k) for k in range(1,5)),2*SCALE)
    # The unconstrained input box overestimates v0 because it loses correlated
    # endpoint hull bounds. The actual analytic derivative is bounded by vl
    # everywhere (the unchanged Bernstein first-difference test).
    vb[0]=1536000*SCALE+error_v*SCALE
    assert all(x<INT64_MAX for x in pb+vb)
    # Also check each addition's operands, not only the mathematical result.
    intermediate=[pb[i]+pb[i+1] for i in range(5)]+[vb[i]+vb[i+1] for i in range(4)]
    assert max(intermediate)<INT64_MAX
    assert pb[0]+SCALE//2<INT64_MAX and vb[0]+SCALE//2<INT64_MAX
    # Canonical q and q+home lie in signed int32 via the hull. Approximation
    # error <1/2 step cannot round beyond an integral representable boundary.
    assert error_q<F(1,2) and error_v<F(1,2)
    assert int(vb[0]/SCALE)+1<1<<31
    return dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        table_bytes=45*8+9,curve_bytes_unchanged=112,
        max_integral_sum=max_whole,max_remainder_sum=max_fractional,
        max_tick_register_bound=str(max(pb+vb)),
        max_tick_addition_bound=str(max(intermediate)),
        q40_position_error_steps=str(error_q),q40_velocity_error_steps_per_s=str(error_v),
        q40_position_error_decimal=float(error_q),q40_velocity_error_decimal=float(error_v),
        rows=bound_rows,note='Exact arithmetic proof; ARM Flash/stack/throughput and hardware validation remain required.'),rows

def write_fixture(path,rows,count=256):
    rng=random.Random(20261014);cases=[]
    explicit=[([0,0,0],[1,0,0]),([0,0,0],[-1,0,0]),
              ([2147483647,0,0],[2147483647,0,0]),
              ([-2147483648,0,0],[-2147483648,0,0])]
    for f,t in explicit:cases.append((f,t,1536000,5120000,0))
    for velocity in [-1536000,-1535950,-51200,-50,-1,0,1,50,51200,1535950,1536000]:
        for acceleration in [-5120000,-5000000,-1000000,0,1000000,5000000,5120000]:
            f=[0,velocity,acceleration]
            t=[velocity//50+acceleration//5000,velocity+acceleration//50,acceleration]
            if accepted(f,t):cases.append((f,t,1536000,5120000,0))
    while len(cases)<count:
        q=rng.randrange(-2000000000,2000000001);v0=rng.randrange(-1536000,1536001)
        v1=v0+rng.randrange(-51200,51201)
        f=[q,v0,rng.randrange(-5120000,5120001)]
        t=[q+int((v0+v1)/100)+rng.randrange(-5,6),v1,rng.randrange(-5120000,5120001)]
        home=rng.randrange(-100000000,100000001)
        if accepted(f,t,home=home):cases.append((f,t,1536000,5120000,home))
    with path.open('wb') as out:
        out.write(struct.pack('<II',0x51463430,len(cases)))
        for f,t,vl,al,home in cases:
            inputs=[t[0]-f[0],f[1],t[1],f[2],t[2]]
            p=[0]+[nearest(sum(c*x for c,x in zip(r['coefficients'],inputs))) for r in rows[:5]]
            v=[f[1]*SCALE]+[nearest(sum(c*x for c,x in zip(r['coefficients'],inputs))) for r in rows[5:]]
            out.write(struct.pack('<9i',*f,*t,vl,al,home))
            out.write(struct.pack('<11q',*p,*v))
            for tick in range(1,401):
                q,velocity=exact_point(f,t,tick)
                out.write(struct.pack('<iidd',f[0]+nearest(q),nearest(velocity),float(q),float(velocity)))
    return len(cases)

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    report,rows=prove(args.source)
    report['exact_fixture_segments']=write_fixture(args.output/'exact_quintic.bin',rows)
    report['exact_fixture_ticks']=400*report['exact_fixture_segments']
    (args.output/'proof.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))

if __name__=='__main__':main()
