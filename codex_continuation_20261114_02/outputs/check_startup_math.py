"""Independent derivative and energy checks for the new prescribed startup motion."""
import math,json
from pathlib import Path
import numpy as np
def motion(t, ramp=.15, f=3.5):
    target=2*math.pi*f
    if t<ramp:
        p=.5*target*(t-ramp/math.pi*math.sin(math.pi*t/ramp))+math.pi/2
        v=.5*target*(1-math.cos(math.pi*t/ramp))
        a=.5*target*math.pi/ramp*math.sin(math.pi*t/ramp)
    else:p=target*(t-.5*ramp)+math.pi/2;v=target;a=0
    return p,v,a
phi=math.pi/4;I=.0049;k=I*(2*math.pi*3.5)**2
def state(t):
    p,v,a=motion(t)
    return phi*math.sin(p),phi*v*math.cos(p),phi*(a*math.cos(p)-v*v*math.sin(p))
errors=[]
for t in (.01,.05,.1,.14,.2,.4):
    eps=1e-6;q,v,a=state(t);qp,vp,_=state(t+eps);qm,vm,_=state(t-eps)
    errors.append(dict(t=t,velocity_error=abs((qp-qm)/(2*eps)-v),acceleration_error=abs((vp-vm)/(2*eps)-a)))
ts=np.linspace(0,.5,50001);states=np.array([state(t) for t in ts]);q,v,a=states.T
# Pair of wings: total KE=I*qdot²; total spring energy=k*q².
work=float(np.trapezoid(2*(I*a+k*q)*v,ts))
delta=float(I*(v[-1]**2-v[0]**2)+k*(q[-1]**2-q[0]**2))
err=abs(work-delta)
assert max(x['velocity_error'] for x in errors)<1e-6
assert max(x['acceleration_error'] for x in errors)<1e-4
assert err<1e-6
result=dict(derivative_checks=errors,pair_signed_work_J=work,pair_energy_change_J=delta,energy_error_J=err,passed=True)
out=Path(__file__).resolve().parents[1]/'outputs'/'phase2'/'math_verification.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result))
