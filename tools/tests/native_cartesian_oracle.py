from pathlib import Path
import argparse
import json
import math
import numpy as np
parser=argparse.ArgumentParser(description='Independent double DH / tool / rail and continuous polynomial oracle; no hardware timing claim.')
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
out=args.output.resolve()
data=json.loads((out/'curves.json').read_text(encoding='utf-8'))
rad=np.pi/180
factor=np.array([1,-1,-1,-1,-1,-1,-1])*np.array([50,50,50,50,50,30,1])*51200/360
factor[6]=-51200/5
offsets=np.array([[0,-.2073,0],[.170,0,0],[-.0695,0,.117],[0,0,0],[0,0,0],[0,0,.113]])
alpha=np.array([-np.pi/2,0,np.pi/2,-np.pi/2,np.pi/2,0])
home=np.array([0,-np.pi/2,np.pi/2,0,0,0])

def fk(q,tool):
    """Independent double DH composition, not the production FK implementation."""
    q=np.atleast_2d(q)
    n=len(q)
    r=np.repeat(np.eye(3)[None,:,:],n,axis=0)
    p=np.zeros((n,3))
    for j in range(6):
        t=q[:,j]*rad+home[j]
        c,s=np.cos(t),np.sin(t)
        ca,sa=np.cos(alpha[j]),np.sin(alpha[j])
        local=np.zeros((n,3,3))
        local[:,0,0]=c;local[:,0,1]=-ca*s;local[:,0,2]=sa*s
        local[:,1,0]=s;local[:,1,1]=ca*c;local[:,1,2]=-sa*c
        local[:,2,1]=sa;local[:,2,2]=ca
        r=np.einsum('nij,njk->nik',r,local)
        p+=np.einsum('nij,j->ni',r,offsets[j])
    p+=np.einsum('nij,j->ni',r,np.array(tool['p']))
    r=np.einsum('nij,jk->nik',r,np.array(tool['r']).reshape(3,3))
    p[:,1]+=q[:,6]*.001
    return p,r

reports=[]
for case in data['cases']:
    if case['result']!='OK':
        reports.append(dict(name=case['name'],result=case['result']))
        continue
    samples=case['samples']
    t=np.array([x[0] for x in samples])
    q=np.array([x[2] for x in samples],dtype=np.float64)/factor+np.array(case['origin'])
    actual,r=fk(q,case['tool'])
    beginq=np.array(case['records'][0],dtype=np.float64)[:,0]/factor+np.array(case['origin'])
    begin,beginR=fk(beginq,case['tool'])
    begin=begin[0];beginR=beginR[0]
    end=np.array(case['goal']['p'])
    delta=end-begin
    duration=case['segments']*.02
    u=t/duration
    s=10*u**3-15*u**4+6*u**5
    wanted=begin+s[:,None]*delta
    wantedR=np.repeat(beginR[None,:,:],len(t),axis=0)
    if case['name'] in ('wrist_turn','wrist180'):
        # Use the declared endpoint's signed shortest yaw. At 180 degrees the
        # two arcs coincide in length; float sine fixes the endpoint side.
        relative_end=np.array(case['goal']['r']).reshape(3,3)@beginR.T
        signed_yaw=math.atan2(relative_end[1,0],relative_end[0,0])
        angle=s*signed_yaw;c,si=np.cos(angle),np.sin(angle)
        rz=np.zeros((len(t),3,3));rz[:,0,0]=rz[:,1,1]=c;rz[:,0,1]=-si;rz[:,1,0]=si;rz[:,2,2]=1
        wantedR=np.einsum('nij,njk->nik',rz,wantedR)
    ep=np.linalg.norm(actual-wanted,axis=1)
    # Stable geodesic angle from skew norm + trace rather than acos near one.
    relative=np.einsum('nji,njk->nik',wantedR,r)
    cosine=np.clip((np.trace(relative,axis1=1,axis2=2)-1)/2,-1,1)
    sine=.5*np.sqrt((relative[:,2,1]-relative[:,1,2])**2+(relative[:,0,2]-relative[:,2,0])**2+(relative[:,1,0]-relative[:,0,1])**2)
    er=np.arctan2(sine,cosine)
    if np.dot(delta,delta)>1e-20:
        closest=np.clip((actual-begin)@delta/np.dot(delta,delta),0,1)
        eline=np.linalg.norm(actual-(begin+closest[:,None]*delta),axis=1)
    else:eline=ep
    # Continuous double polynomial derivatives from the protocol boundary data.
    records=np.array(case['records'],dtype=np.float64)
    fromq,toq=records[:-1,:,0],records[1:,:,0]
    fromv,tov=records[:-1,:,1],records[1:,:,1]
    froma,toa=records[:-1,:,2],records[1:,:,2]
    # Solve the endpoint boundary-value system independently rather than using
    # the production coefficient expressions. Work in displacement coordinates
    # to avoid cancellation of a large motor home offset.
    boundary=np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],
                       [1,1,1,1,1,1],[0,1,2,3,4,5],[0,0,2,6,12,20]],dtype=np.float64)
    rhs=np.stack([np.zeros_like(fromq),fromv*.02,froma*.02**2,
                  toq-fromq,tov*.02,toa*.02**2],axis=2)
    c=np.linalg.solve(boundary,rhs.reshape(-1,6).T).T.reshape(rhs.shape)
    c[:,:,0]+=fromq
    vc=c[:,:,1:]*np.arange(1,6)[None,None,:]/.02/factor[None,:,None]
    ac=c[:,:,2:]*np.array([2,6,12,20])[None,None,:]/.02**2/factor[None,:,None]
    jc=c[:,:,3:]*np.array([6,24,60])[None,None,:]/.02**3/factor[None,:,None]
    def continuous_max(polynomials):
        shape=polynomials.shape
        maxima=[]
        for coefficients in polynomials.reshape(-1,shape[-1]):
            derivative=coefficients[1:]*np.arange(1,len(coefficients))
            candidates=[0.,1.]
            if np.any(derivative):
                roots=np.roots(derivative[::-1])
                candidates.extend(float(z.real) for z in roots if abs(z.imag)<1e-7 and 0<z.real<1)
            maxima.append(max(abs(float(np.polynomial.polynomial.polyval(t,coefficients))) for t in candidates))
        return np.max(np.array(maxima).reshape(shape[:2]),axis=0)
    vmax,amax,jmax=continuous_max(vc),continuous_max(ac),continuous_max(jc)
    vl=np.array([40]*6+[20]);al=np.array([80]*6+[40]);jl=np.array([20000]*6+[3000])
    report=dict(name=case['name'],result='OK',segments=case['segments'],samples=len(samples),rail=case['rail'],
      position_error_mm_max=float(ep.max()*1000),line_error_mm_max=float(eline.max()*1000),rotation_error_deg_max=float(er.max()/rad),
      reported_bound_mm=case['bound_m']*1000,reported_bound_deg=case['bound_rad']/rad,
      motor_curve_error_steps_max=case['motor_curve_error_steps_max'],motor_velocity_error_steps_per_s_max=case['motor_velocity_error_steps_per_s_max'],
      velocity_max=vmax.tolist(),acceleration_max=amax.tolist(),jerk_max=jmax.tolist(),
      position_bound_respected=bool(ep.max()<=case['bound_m']+2e-6),rotation_bound_respected=bool(er.max()<=case['bound_rad']+2e-6),
      nominal_1mm_1deg_respected=bool(ep.max()<=.001 and er.max()<=rad),dynamic_respected=bool(np.all(vmax<=vl+1e-4) and np.all(amax<=al+1e-3) and np.all(jmax<=jl+1)),integer_curve_respected=bool(case['motor_curve_error_steps_max']<=2 and case['motor_velocity_error_steps_per_s_max']<=2))
    reports.append(report)
expected={'near_singular':'DISCONTINUITY','rail450':'JOINT_LIMIT'}
contracts=(data['accepted']==10 and len(data['cases'])==12 and data['invalid_velocity_result']=='INVALID' and data['cancelled_result']=='CANCELLED' and not data['cancelled_Record_allowed'] and all(c['result']==expected.get(c['name'],'OK') for c in data['cases']))
report=dict(passed=contracts and all(x.get('nominal_1mm_1deg_respected',True) and x.get('dynamic_respected',True) and x.get('position_bound_respected',True) and x.get('rotation_bound_respected',True) and x.get('integer_curve_respected',True) for x in reports),
  scope='Seven-axis quantized production motor finite-difference samples; independent double DH / tool / rail transform and continuous encoded-polynomial derivative oracle. Sampled geometric evidence, not proof of all times or physical tracking; CMSIS shim not ARM timing.',
  dynamic_method='Independent6x6 boundary-value solve; endpoints and real stationary roots for continuous velocity/acceleration/jerk maxima.',cases=reports,public_contract_probes={k:data[k] for k in ('invalid_velocity_result','cancelled_result','cancelled_Record_allowed')})
(out/'oracle.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
accepted=[x for x in reports if x['result']=='OK']
print(json.dumps(dict(passed=report['passed'],accepted=len(accepted),rejected=len(reports)-len(accepted),samples=sum(x['samples'] for x in accepted),max_position_error_mm=max(x['position_error_mm_max'] for x in accepted),max_rotation_error_deg=max(x['rotation_error_deg_max'] for x in accepted),max_curve_error_steps=max(x['motor_curve_error_steps_max'] for x in accepted),max_velocity_error_steps_per_s=max(x['motor_velocity_error_steps_per_s_max'] for x in accepted),report=str(out/'oracle.json')),ensure_ascii=False))

raise SystemExit(0 if report['passed'] else 1)
