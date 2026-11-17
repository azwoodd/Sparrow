"""Isolated finite-startup trajectories and inverse-dynamics actuator feasibility gates.
No saturated flight dynamics, real motor selection, or physical validation is claimed.
"""
import os, sys, math, json, time, inspect, difflib, subprocess, ctypes
from pathlib import Path
from ctypes import wintypes
import guarded_study as guard
ROOT=guard.ROOT
OUT=Path(__file__).resolve().parents[1]/'outputs'/('phase2_actuator_check' if os.environ.get('SPARROW_ACTUATOR_CHECK')=='1' else ('phase2_actuator' if os.environ.get('SPARROW_ACTUATOR')=='1' else ('phase2_timing' if os.environ.get('SPARROW_STARTUP_TIMING')=='1' else 'phase2')))
if os.environ.get('SPARROW_ACTUATOR_LONG')=='1':OUT=OUT.parent/'phase2_actuator_long'
OUT.mkdir(parents=True,exist_ok=True)
def processes():
    class Entry(ctypes.Structure):
        _fields_=[('size',wintypes.DWORD),('usage',wintypes.DWORD),('pid',wintypes.DWORD),('heap',ctypes.c_size_t),('module',wintypes.DWORD),('threads',wintypes.DWORD),('parent',wintypes.DWORD),('priority',wintypes.LONG),('flags',wintypes.DWORD),('name',wintypes.WCHAR*260)]
    k=ctypes.WinDLL('kernel32',use_last_error=True)
    k.CreateToolhelp32Snapshot.restype=wintypes.HANDLE
    k.Process32FirstW.argtypes=[wintypes.HANDLE,ctypes.POINTER(Entry)]
    k.Process32NextW.argtypes=k.Process32FirstW.argtypes
    k.CloseHandle.argtypes=[wintypes.HANDLE]
    handle=k.CreateToolhelp32Snapshot(2,0)
    if handle==ctypes.c_void_p(-1).value:raise ctypes.WinError()
    e=Entry();e.size=ctypes.sizeof(e);result=[]
    try:
        ok=k.Process32FirstW(handle,ctypes.byref(e))
        while ok:
            if 'python' in e.name.lower():result.append(dict(pid=e.pid,parent=e.parent,name=e.name,threads=e.threads))
            ok=k.Process32NextW(handle,ctypes.byref(e))
    finally:k.CloseHandle(handle)
    return result
def worker():
    if os.environ.get('SPARROW_ACTUATOR')=='1':
        import actuator_study
        actuator_study.main(OUT)
        return
    import numpy as np
    sys.path.insert(0,str(ROOT/'flight_research'))
    import f1_wing_model as w
    import f3_jump_to_flight as f3
    original=w.wing_forces;source=inspect.getsource(original)
    patched=source.replace('    w = 2 * math.pi * f\n    phi = PHI * math.sin(w * t); dphi = PHI * w * math.cos(w * t)',
        '    phase, omega, alpha = MOTION(t, f)\n    phi = PHI * math.sin(phase); dphi = PHI * omega * math.cos(phase)\n    ddphi = PHI * (alpha * math.cos(phase) - omega**2 * math.sin(phase))')
    patched=patched.replace('math.cos(w * t)','math.cos(phase)').replace('-PHI * w * w * math.sin(w * t)','ddphi')
    patched=patched.replace('    # induced drag of the finite wing','    if MIRROR_RESULTANT: tot[0] = 0.0\n    # induced drag of the finite wing')
    assert patched!=source and 'w * t' not in patched
    (OUT/'startup_model.diff').write_text(''.join(difflib.unified_diff(source.splitlines(True),patched.splitlines(True),fromfile='original_wing_forces',tofile='finite_startup_experiment')))
    exec(compile(patched,'finite_startup_experiment','exec'),w.__dict__)
    modified=w.wing_forces
    # Exact zero-ramp compatibility at several nontrivial phases/velocities.
    w.MIRROR_RESULTANT=False;w.MOTION=lambda t,f:(2*math.pi*f*t,2*math.pi*f,0)
    errors=[]
    for t in (.001,.03,.11,.24):
        a=original(t,3.5,.8,.07,.5,(4.,1.),1.6)
        b=modified(t,3.5,.8,.07,.5,(4.,1.),1.6)
        errors.append(max(float(np.max(np.abs(np.asarray(x)-np.asarray(y)))) for x,y in zip(a,b)))
    assert max(errors)<1e-12
    print('Force-function compatibility error',max(errors),flush=True)
    rows=[]
    def run(stage,mass=.529,area=3.,f=3.5,ramp=0.,phase0=0.,eff=.8,cn=1.6,mirror=False,fixed_body=False,dt=.002,T=15.,delay=.15):
        phi_deg=45;E=13.;tw=50.;b0=40.;be=15.
        f3.set_wing(f3.R_SP,f3.A_SP*area,f3.SPAN_SP,mass,phi_deg,f3.I_SP)
        if fixed_body:w.BODY_CDA=.05*float(2*f3.A_SP.sum())
        w.ETA_S=.7;w.MIRROR_RESULTANT=mirror
        def motion(t,freq):
            target=2*math.pi*freq
            if t<0:return math.radians(phase0),0.,0.
            if ramp>0 and t<ramp:
                omega=.5*target*(1-math.cos(math.pi*t/ramp))
                alpha=.5*target*math.pi/ramp*math.sin(math.pi*t/ramp)
                phase=.5*target*(t-ramp/math.pi*math.sin(math.pi*t/ramp))
            else:omega=target;alpha=0.;phase=target*(t-.5*ramp)
            return phase+math.radians(phase0),omega,alpha
        w.MOTION=motion
        vx,vz=f3.jump(E,mass,45,eff);vx0,vz0=vx,vz
        vs=math.sqrt(2*mass*w.G/(1.225*w.S_WING));h=0.;hs=[]
        k_tuned=f3.I_SP*(2*math.pi*f)**2
        accum={name:dict(peak_Nm=0.,sq_Nm=0.,positive_J=0.,signed_J=0.,peak_W=0.,over_torque_s=0.,over_peak_power_s=0.,peak_crank_Nm=0.) for name in ('no_spring','ideal_tuned_spring')}
        min_clearance=1e9;collision_at=None;elapsed=0.;aero_j=0.;samples=0
        initial_ke=f3.I_SP*(w.PHI*motion(0,f)[1]*math.cos(math.radians(phase0)))**2
        # Zero-ramp/phase-zero cases have unmodelled impulsive startup; record rather than certify.
        start=time.perf_counter();first_active=None;last_energy={}
        for index in range(int(round(T/dt))):
            t=index*dt;beta=math.radians(be+(b0-be)*max(0.,1-math.hypot(vx,vz)/(1.5*vs)))
            tau=t-delay
            phase,omega,alpha=motion(tau,f)
            phi=w.PHI*math.sin(phase)
            F,p,ta,dphi,ddphi=modified(tau,f,math.radians(tw) if tau>=0 else 0.,math.radians(4),beta,(vx,vz),cn)
            # Centerline-only clearance proxy, using copied standing base/hinge offsets.
            root_z=.165+h-.06*math.sin(beta)+.07*math.cos(beta)
            clearance=root_z+min(0.,w.R*math.sin(phi)*math.cos(beta))
            min_clearance=min(min_clearance,clearance)
            if clearance<0 and collision_at is None:collision_at=t
            if tau>=0:
                samples+=1;aero_j+=max(p,0.)*dt
                if first_active is None:first_active=t
                for name,k in (('no_spring',0.),('ideal_tuned_spring',k_tuned)):
                    tm=f3.I_SP*ddphi-ta+k*phi
                    power=2*tm*dphi;crank=2*tm*w.PHI*math.cos(phase)
                    a=accum[name];a['peak_Nm']=max(a['peak_Nm'],abs(tm));a['sq_Nm']+=tm*tm
                    a['positive_J']+=max(power,0.)*dt;a['signed_J']+=power*dt;a['peak_W']=max(a['peak_W'],max(power,0.))
                    a['over_torque_s']+=(abs(tm)>4.)*dt;a['over_peak_power_s']+=(power>100.)*dt
                    a['peak_crank_Nm']=max(a['peak_crank_Nm'],abs(crank))
                    last_energy[name]=f3.I_SP*dphi*dphi+k*phi*phi
            D=w.body_drag((vx,vz));vx+=(F[0]+D[0])/mass*dt;vz+=((F[1]+D[1])/mass-w.G)*dt;h+=vz*dt
            elapsed=(index+1)*dt;hs.append(h)
            if h<0 and elapsed>.1:break
        completed=elapsed>=T-dt/2
        sink=(hs[-1]-hs[-int(round(1/dt))]) if len(hs)>int(round(1/dt)) else -999
        airborne=bool(completed and sink>-.15)
        active=max(samples*dt,dt)
        for name,k in (('no_spring',0.),('ideal_tuned_spring',k_tuned)):
            a=accum[name];a['mean_positive_W']=a['positive_J']/active;a['rms_Nm']=math.sqrt(a.pop('sq_Nm')/max(samples,1))
            a['spring_k_Nm_rad']=k;a['initial_spring_J']=k*(w.PHI*math.sin(math.radians(phase0)))**2
            a['load_gate']=bool(a['peak_Nm']<=4 and a['mean_positive_W']<=50 and a['peak_W']<=100)
            a['candidate_gate']=bool(airborne and collision_at is None and a['load_gate'] and ramp>0)
        rec=dict(stage=stage,mass=mass,area=area,f=f,stroke_deg=45,E_J=13,twist_deg=50,cn=cn,eta=.7,eff=eff,ramp_s=ramp,phase0_deg=phase0,delay_s=delay,mirror=mirror,fixed_body=fixed_body,dt=dt,T=T,elapsed=elapsed,airborne=airborne,forward=bool(airborne and vx>=vs),vx0=vx0,vz0=vz0,end_vx=vx,end_vz=vz,end_h=h,last_second_climb=sink,minimum_centerline_clearance=min_clearance,first_centerline_collision=collision_at,aero_positive_W=aero_j/active,metrics=accum,initial_unmodelled_kinetic_J=initial_ke if ramp==0 else 0.,wall_s=time.perf_counter()-start)
        rows.append(rec)
        with (OUT/'startup_runs.jsonl').open('a') as out:out.write(json.dumps(rec)+'\n')
        return rec
    if os.environ.get('SPARROW_STARTUP_TIMING')=='1':
        for mass in (.529,.559):
            for f in (3.5,4.):
                for ramp in (.1,.2):
                    for delay in (0.,.05,.1,.15):
                        for eff in (.5,.8):
                            for cn in (1.2,1.6):
                                for fixed in (False,True):
                                    run('timing_screen',mass=mass,f=f,ramp=ramp,phase0=90,delay=delay,eff=eff,cn=cn,fixed_body=fixed)
            print('Timing mass complete',mass,len(rows),flush=True)
        candidates=[r for r in rows if r['airborne'] and r['first_centerline_collision'] is None]
        candidates.sort(key=lambda r:(not r['metrics']['ideal_tuned_spring']['load_gate'],r['eff'],r['metrics']['ideal_tuned_spring']['peak_W']))
        selected=[]
        for r in candidates:
            if not any(all(r[k]==q[k] for k in ('mass','eff','cn','fixed_body')) for q in selected):selected.append(r)
            if len(selected)>=6:break
        for r in selected:
            for dt in (.001,.0005):
                run('timing_convergence',mass=r['mass'],f=r['f'],ramp=r['ramp_s'],phase0=90,delay=r['delay_s'],eff=r['eff'],cn=r['cn'],fixed_body=r['fixed_body'],dt=dt)
        (OUT/'results_summary.json').write_text(json.dumps(dict(count=len(rows),airborne=sum(r['airborne'] for r in rows),spring_gate=sum(r['metrics']['ideal_tuned_spring']['candidate_gate'] for r in rows)),indent=2))
        return
    # Establish force-formulation sensitivity independently of spin-up.
    for mirror in (False,True):
        for fixed in (False,True):
            run('formulation',mass=.559,mirror=mirror,fixed_body=fixed,dt=.001)
    # Original force closure retained so startup effects are not mixed with model fixes.
    for mass,area,f in ((.529,3.,3.5),(.559,2.5,4.),(.559,3.,4.)):
        for ramp in (0.,.15,.3,.5):
            for phase in (0.,90.):
                for eff in (.5,.8):
                    for cn in (1.2,1.6):
                        run('startup_screen',mass=mass,area=area,f=f,ramp=ramp,phase0=phase,eff=eff,cn=cn)
        print('Completed startup family',mass,area,f,len(rows),flush=True)
    # Recheck load-gate candidates and at least the best finite-startup survivor.
    candidates=[r for r in rows if r['stage']=='startup_screen' and r['airborne'] and r['ramp_s']>0 and r['first_centerline_collision'] is None]
    candidates.sort(key=lambda r:r['metrics']['ideal_tuned_spring']['mean_positive_W'])
    chosen=[]
    for r in candidates:
        if not any(all(r[k]==q[k] for k in ('mass','area','f','eff','cn')) for q in chosen):chosen.append(r)
        if len(chosen)>=4:break
    for r in chosen:
        for dt in (.001,.0005):
            run('startup_convergence',mass=r['mass'],area=r['area'],f=r['f'],ramp=r['ramp_s'],phase0=r['phase0_deg'],eff=r['eff'],cn=r['cn'],dt=dt)
    summary=dict(count=len(rows),force_compatibility_max_error=max(errors),finite_startup_airborne=len(candidates),no_spring_candidates=sum(r['metrics']['no_spring']['candidate_gate'] for r in rows),ideal_spring_candidates=sum(r['metrics']['ideal_tuned_spring']['candidate_gate'] for r in rows))
    (OUT/'results_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
def main():
    before=guard.hashes();(OUT/'source_hashes.json').write_text(json.dumps(before,indent=2))
    procs=processes();(OUT/'processes_before.json').write_text(json.dumps(procs,indent=2))
    # Conservatively refuse new physics if ANY unrelated Python executable is running.
    ancestors={os.getpid(),os.getppid()};unrelated=[p for p in procs if p['pid'] not in ancestors]
    if unrelated:raise RuntimeError('Unclassified Python processes; inspect before simulation: '+str(unrelated))
    last=guard.ticks();time.sleep(1);last,u=guard.usage(last)
    if max(u['cpu'],u['ram'])>=80:raise RuntimeError('Insufficient headroom '+str(u))
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    start=time.monotonic();stop=None
    with (OUT/'study.log').open('w') as log,(OUT/'resources.jsonl').open('w') as monitor:
        child=subprocess.Popen([sys.executable,'-B',__file__,'worker'],env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.BELOW_NORMAL_PRIORITY_CLASS)
        try:
            while child.poll() is None:
                time.sleep(1);last,u=guard.usage(last);u['elapsed_s']=round(time.monotonic()-start,1);monitor.write(json.dumps(u)+'\n');monitor.flush()
                if max(u['cpu'],u['ram'])>=85 or time.monotonic()-start>600:
                    stop='resource threshold' if max(u['cpu'],u['ram'])>=85 else 'ten minute limit';child.terminate();child.wait(timeout=10);break
        finally:
            if child.poll() is None:child.terminate();child.wait(timeout=10)
    result=dict(exit_code=child.returncode,stop_reason=stop,sources_unchanged=before==guard.hashes(),wall_s=time.monotonic()-start)
    (OUT/'guard_summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':
    worker() if len(sys.argv)>1 and sys.argv[1]=='worker' else main()
