"""Ideal torque/power-limited hinge dynamics; prescribed twist/pitch remain explicit limitations."""
import math,sys,inspect,json,time,os
from pathlib import Path
import numpy as np
ROOT=Path(r'D:\Downloads\Sparrow_Locomotion_Engineering_V1')
def main(out):
    sys.path.insert(0,str(ROOT/'flight_research'))
    import f1_wing_model as w
    import f3_jump_to_flight as f3
    source=inspect.getsource(w.wing_forces)
    patched=source.replace('    w = 2 * math.pi * f\n    phi = PHI * math.sin(w * t); dphi = PHI * w * math.cos(w * t)',
        '    phi, dphi, phase = ACTUAL_STATE')
    patched=patched.replace('math.cos(w * t)','math.cos(phase)').replace('-PHI * w * w * math.sin(w * t)','0.0')
    exec(compile(patched,'actual_wing_state_force','exec'),w.__dict__)
    (out/'actual_state_force.py').write_text(patched)
    def run(cap=100.,torque=4.,kp=60.,mass=.529,cn=1.6,eff=.8,dt=.0005,T=15.,stage='actuator_screen',spring_scale=1.,inertia_scale=1.):
        f=3.5;area=3.;ramp=.1;kd=2*math.sqrt(kp*f3.I_SP);nominal_k=f3.I_SP*(2*math.pi*f)**2;k=nominal_k*spring_scale;actual_I=f3.I_SP*inertia_scale
        f3.set_wing(f3.R_SP,f3.A_SP*area,f3.SPAN_SP,mass,45,actual_I);w.ETA_S=.7
        vx,vz=f3.jump(13,mass,45,eff);h=0.;q=w.PHI;dq=0.;hs=[]
        vs=math.sqrt(2*mass*w.G/(1.225*w.S_WING))
        maxerr=0.;sumerr=0.;pJ=0.;maxp=0.;maxt=0.;t2=0.;sat=0.;clearmin=1e9;collide=None
        maxspeed=0.;maxstroke=0.;aeroJ=0.;start=time.perf_counter();trace=[]
        for i in range(round(T/dt)):
            t=i*dt;omega0=2*math.pi*f
            if t<ramp:
                omega=.5*omega0*(1-math.cos(math.pi*t/ramp));alpha=.5*omega0*math.pi/ramp*math.sin(math.pi*t/ramp)
                phase=math.pi/2+.5*omega0*(t-ramp/math.pi*math.sin(math.pi*t/ramp))
            else:omega=omega0;alpha=0.;phase=math.pi/2+omega0*(t-.5*ramp)
            qd=w.PHI*math.sin(phase);dqd=w.PHI*omega*math.cos(phase);ddqd=w.PHI*(alpha*math.cos(phase)-omega**2*math.sin(phase))
            beta=math.radians(15+(40-15)*max(0.,1-math.hypot(vx,vz)/(1.5*vs)))
            w.ACTUAL_STATE=(q,dq,phase)
            F,p,ta,_,_=w.wing_forces(t,f,math.radians(50),math.radians(4),beta,(vx,vz),cn)
            # Known inertia/spring feedforward plus PD feedback; NO aerodynamic oracle feedforward.
            requested=f3.I_SP*ddqd+nominal_k*qd+kp*(qd-q)+kd*(dqd-dq)
            tau=max(-torque,min(torque,requested))
            if 2*tau*dq>cap:tau=cap/(2*dq)
            sat+=abs(tau-requested)>1e-9
            power=max(2*tau*dq,0.);pJ+=power*dt;aeroJ+=max(p,0.)*dt;maxp=max(maxp,power);maxt=max(maxt,abs(tau));t2+=tau*tau
            error=abs(q-qd);maxerr=max(maxerr,error);sumerr+=error*error
            maxspeed=max(maxspeed,abs(dq));maxstroke=max(maxstroke,abs(q))
            root=.165+h-.06*math.sin(beta)+.07*math.cos(beta)
            clearance=root+min(0.,w.R*math.sin(q)*math.cos(beta));clearmin=min(clearmin,clearance)
            if clearance<0 and collide is None:collide=t
            ddq=(tau+ta-k*q)/actual_I
            dq+=ddq*dt;q+=dq*dt
            D=w.body_drag((vx,vz));vx+=(F[0]+D[0])/mass*dt;vz+=((F[1]+D[1])/mass-w.G)*dt;h+=vz*dt;hs.append(h)
            if i%100==0:trace.append([t,h,vx,vz,q,dq,qd,tau,power])
            if h<0 and t>.1:break
            if abs(q)>math.radians(70):break # explicit travel failure, not a hidden clamp
        elapsed=(i+1)*dt;climb=hs[-1]-hs[-round(1/dt)] if len(hs)>round(1/dt) else -999
        flight=elapsed>=T-dt/2 and climb>-.15 and collide is None
        rec=dict(stage=stage,mass=mass,cn=cn,eff=eff,f=3.5,area=3.,ramp_s=.1,phase0_deg=90,delay_s=0,E_J=13,spring_k=k,spring_scale=spring_scale,inertia_scale=inertia_scale,actual_wing_inertia=actual_I,initial_spring_J=k*w.PHI*w.PHI,kp=kp,kd=kd,torque_limit_per_wing_Nm=torque,positive_power_limit_both_W=cap,dt=dt,T=T,elapsed=elapsed,airborne=bool(flight),forward=bool(flight and vx>=vs),end_h=h,end_vx=vx,end_vz=vz,climb_last_second=climb,mean_positive_mech_W=pJ/elapsed,peak_positive_mech_W=maxp,peak_actual_torque_Nm=maxt,rms_actual_torque_Nm=math.sqrt(t2/(i+1)),mean_positive_aero_W=aeroJ/elapsed,peak_tracking_error_deg=math.degrees(maxerr),rms_tracking_error_deg=math.degrees(math.sqrt(sumerr/(i+1))),saturation_fraction=sat/(i+1),minimum_centerline_clearance=clearmin,collision_time=collide,peak_flap_speed_rad_s=maxspeed,peak_stroke_deg=math.degrees(maxstroke),wall_s=time.perf_counter()-start)
        with (out/'actuator_runs.jsonl').open('a') as fobj:fobj.write(json.dumps(rec)+'\n')
        if stage=='actuator_convergence':
            name=f"trace_mass{int(mass*1000)}_dt{dt}.csv"
            np.savetxt(out/name,np.asarray(trace),delimiter=',',header='t_s,h_m,vx_m_s,vz_m_s,wing_rad,wing_rad_s,command_rad,motor_hinge_Nm,pair_mech_W',comments='')
        return rec
    rows=[]
    if os.environ.get('SPARROW_ACTUATOR_LONG')=='1':
        for dt in (.0005,.00025):
            rows.append(run(cap=80,torque=2.5,kp=40,dt=dt,T=60,stage='long_low_envelope'))
            rows.append(run(mass=.559,spring_scale=.8,dt=dt,T=60,stage='long_soft_spring'))
        for cn,eff in ((1.2,.8),(1.4,.8),(1.6,.7)):
            rows.append(run(mass=.559,spring_scale=.8,cn=cn,eff=eff,T=30,stage='soft_spring_uncertainty'))
        (out/'results_summary.json').write_text(json.dumps(dict(count=len(rows),airborne=sum(r['airborne'] for r in rows)),indent=2))
        return
    if os.environ.get('SPARROW_ACTUATOR_CHECK')=='1':
        for dt in (.001,.0005,.00025):rows.append(run(mass=.559,dt=dt,T=30,stage='actuator_convergence'))
        rows.append(run(cap=80,torque=2.5,kp=40,dt=.00025,T=30,stage='low_envelope_convergence'))
        for cn,eff in ((1.2,.8),(1.4,.8),(1.6,.7),(1.6,.6),(1.6,.5)):
            rows.append(run(mass=.559,cn=cn,eff=eff,T=30,stage='actuator_uncertainty'))
        for ks,Is in ((.8,1.),(1.2,1.),(1.,1.25),(.8,1.25)):
            rows.append(run(mass=.559,spring_scale=ks,inertia_scale=Is,T=30,stage='mechanical_uncertainty'))
        rows.append(run(mass=.589,T=30,stage='mass_allowance'))
        (out/'results_summary.json').write_text(json.dumps(dict(count=len(rows),airborne=sum(r['airborne'] for r in rows)),indent=2))
        return
    for cap in (80.,100.,120.):
        for tq in (2.5,4.):
            for kp in (40.,60.):rows.append(run(cap=cap,torque=tq,kp=kp))
        print('Power-envelope complete',cap,flush=True)
    for mass,cn,eff in ((.529,1.2,.8),(.529,1.6,.5),(.559,1.6,.8),(.559,1.2,.5)):
        rows.append(run(mass=mass,cn=cn,eff=eff))
    # Convergence for reference actuator even when it fails; no success-only reporting.
    for dt in (.001,.00025):rows.append(run(dt=dt,stage='actuator_convergence'))
    (out/'results_summary.json').write_text(json.dumps(dict(count=len(rows),airborne=sum(r['airborne'] for r in rows)),indent=2))
