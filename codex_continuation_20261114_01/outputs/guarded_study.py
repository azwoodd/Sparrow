"""Read-only imports of Sparrow; isolated outputs; one guarded experiment worker."""
import os, sys, json, time, math, ctypes, subprocess, hashlib, itertools
from pathlib import Path
from ctypes import wintypes
ROOT=Path(r'D:\Downloads\Sparrow_Locomotion_Engineering_V1')
OUT=Path(os.environ.get('SPARROW_OUTPUT_DIR',str(Path(__file__).resolve().parents[1]/'outputs')))
OUT.mkdir(parents=True,exist_ok=True)
class Memory(ctypes.Structure):
    _fields_=[('length',wintypes.DWORD),('load',wintypes.DWORD)]+[(x,ctypes.c_ulonglong) for x in ['totalPhys','availPhys','totalPage','availPage','totalVirtual','availVirtual','availExtended']]
K=ctypes.WinDLL('kernel32',use_last_error=True)
def ticks():
    a,b,c=(wintypes.FILETIME() for _ in range(3))
    if not K.GetSystemTimes(ctypes.byref(a),ctypes.byref(b),ctypes.byref(c)): raise ctypes.WinError()
    return tuple((x.dwHighDateTime<<32)+x.dwLowDateTime for x in (a,b,c))
def usage(last):
    now=ticks(); d=[y-x for x,y in zip(last,now)]
    m=Memory();m.length=ctypes.sizeof(m)
    if not K.GlobalMemoryStatusEx(ctypes.byref(m)): raise ctypes.WinError()
    return now,dict(cpu=round(100*(1-d[0]/max(1,d[1]+d[2])),2),ram=m.load,available_gb=round(m.availPhys/1024**3,2))
def hashes():
    paths=list((ROOT/'flight_research').glob('*.py'))+list((ROOT/'model'/'v7e_stiffcontact').glob('*.xml'))+list((ROOT/'model'/'v7e_stiffcontact').glob('*.json'))
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
def worker():
    sys.path.insert(0,str(ROOT/'flight_research'))
    import f1_wing_model as w
    import f3_jump_to_flight as f3
    orig=w.wing_forces
    rows=[]
    def run(stage,area=2,mass=.529,phi=45,E=13,f=4.5,tw=50,b0=40,be=15,cn=1.6,eta=.7,dt=.002,T=5,eff=.8):
        f3.set_wing(f3.R_SP,f3.A_SP*area,f3.SPAN_SP,mass,phi,f3.I_SP)
        f3.CN=cn;w.ETA_S=eta
        sums={str(s):[0.,0.,0.,0.] for s in (1.,.5,.25)}; samples=[0]
        def measured(t,hz,*args):
            result=orig(t,hz,*args)
            F,p,ta,dp,ddp=result
            if hz>1e-5:
                samples[0]+=1
                for scale in (1.,.5,.25):
                    tq=f3.I_SP*scale*ddp-ta
                    a=sums[str(scale)];a[0]=max(a[0],abs(tq));a[1]+=max(2*tq*dp,0);a[2]+=tq*tq;a[3]=max(a[3],max(2*tq*dp,0))
            return result
        w.wing_forces=measured
        vs=math.sqrt(2*mass*w.G/(1.225*w.S_WING));vx,vz=f3.jump(E,mass,45,eff=eff)
        start=time.perf_counter()
        ok,hmin,end,P,tq=f3.fly(vx,vz,f,tw,b0,be,1.5*vs,T=T,dt=dt)
        w.wing_forces=orig
        metrics={s:dict(peak_torque_per_wing_Nm=a[0],positive_mech_both_W=a[1]/max(samples[0],1),rms_torque_per_wing_Nm=math.sqrt(a[2]/max(samples[0],1)),peak_positive_mech_both_W=a[3]) for s,a in sums.items()}
        rec=dict(stage=stage,area=area,mass=mass,phi=phi,E=E,f=f,tw=tw,b0=b0,be=be,cn=cn,eta=eta,dt=dt,T=T,eff=eff,delay=.15,angle=45,ok=bool(ok),forward_ok=bool(ok and end[1]>=vs),hmin=hmin,end=end,aero_W=P,torque_Nm=tq,metrics=metrics,wall_s=time.perf_counter()-start,wing_area_m2=w.S_WING,span_m=w.SPAN,vx0=vx,vz0=vz)
        rows.append(rec)
        with (OUT/'flight_runs.jsonl').open('a') as out:out.write(json.dumps(rec)+'\n')
        return rec
    if os.environ.get('SPARROW_SYMMETRY')=='1':
        import inspect, difflib
        source=inspect.getsource(orig)
        needle='    tot = 2 * F.sum(axis=0)'
        assert source.count(needle)==1
        patched=source.replace('    # induced drag of the finite wing', '    tot[0] = 0.0  # mirrored wing cancels lateral force before induced-drag estimate\n    # induced drag of the finite wing')
        (OUT/'symmetry_experiment.diff').write_text(''.join(difflib.unified_diff(source.splitlines(True),patched.splitlines(True),fromfile='original_wing_forces',tofile='isolated_symmetry_variant')))
        original=orig
        for corrected in (False,True):
            if corrected:
                exec(compile(patched,'isolated_symmetry_variant','exec'),w.__dict__)
                orig=w.wing_forces
            else:orig=original
            for mass,area,f in ((.529,2.,4.5),(.529,3.,3.5),(.559,3.,3.5),(.559,3.,4.)):
                for cn in (1.2,1.6):
                    run('symmetry_corrected' if corrected else 'symmetry_original',mass=mass,area=area,f=f,cn=cn,T=15,dt=.001)
        return
    if os.environ.get('SPARROW_FOLLOWUP')=='1':
        previous=[json.loads(s) for s in (OUT/'flight_runs.jsonl').read_text().splitlines()]
        candidates=[]
        for mass in (.529,.559,.589):
            pool=sorted([r for r in previous if r['stage']=='screen' and r['forward_ok'] and r['mass']==mass],key=lambda r:r['aero_W'])
            candidates.extend(pool[:3])
        for candidate in candidates:
            cfg={k:candidate[k] for k in ('area','mass','phi','E','f','tw','b0','be')}
            checks=[run('extended_convergence',**cfg,T=15,dt=dt) for dt in (.002,.001,.0005)]
            if all(r['forward_ok'] for r in checks):
                for cn,eta,tw in itertools.product((1.2,2.),(.5,.9),(35,50,65)):
                    run('extended_uncertainty',**{**cfg,'tw':tw},cn=cn,eta=eta,T=15,dt=.001)
                run('extended_launch',**cfg,T=15,dt=.001,eff=.5)
            print('Extended candidate complete',json.dumps(cfg),flush=True)
        return
    baseline=run('baseline')
    print('Baseline',json.dumps(baseline),flush=True)
    for area,mass,phi,E,f in itertools.product((2.,2.5,3.),(.529,.559,.589),(45,60),(10,13),(3.5,4.,4.5)):
        run('screen',area=area,mass=mass,phi=phi,E=E,f=f)
        if len(rows)%18==0:print('completed',len(rows),'cases',flush=True)
    viable=[r for r in rows if r['stage']=='screen' and r['forward_ok'] and r['mass']>=.559]
    if viable:
        best=min(viable,key=lambda r:r['aero_W'])
        cfg={k:best[k] for k in ('area','mass','phi','E','f','tw','b0','be')}
        print('Selected sensitivity candidate',json.dumps(cfg),flush=True)
        for cn,eta,tw in itertools.product((1.2,1.6,2.),(.5,.7,.9),(35,50,65)):
            run('sensitivity',**{**cfg,'tw':tw},cn=cn,eta=eta)
        run('dt_check',**cfg,dt=.001)
        run('duration_check',**cfg,T=15)
        run('launch_efficiency_check',**cfg,eff=.5)
        (OUT/'selected_candidate.json').write_text(json.dumps(best,indent=2))
    print('FINISHED',len(rows),flush=True)
def supervise():
    before=hashes();(OUT/'source_hashes_before.json').write_text(json.dumps(before,indent=2))
    last=ticks();time.sleep(1);last,u=usage(last)
    if max(u['cpu'],u['ram'])>=80:raise RuntimeError('Insufficient headroom '+str(u))
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    start=time.monotonic();stop=None
    suffix='_symmetry' if os.environ.get('SPARROW_SYMMETRY')=='1' else ('_followup' if os.environ.get('SPARROW_FOLLOWUP')=='1' else '')
    with (OUT/('study'+suffix+'.log')).open('w') as log,(OUT/('resources'+suffix+'.jsonl')).open('w') as monitor:
        child=subprocess.Popen([sys.executable,'-B',__file__,'worker'],env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.BELOW_NORMAL_PRIORITY_CLASS)
        try:
            while child.poll() is None:
                time.sleep(1);last,u=usage(last);u['elapsed_s']=round(time.monotonic()-start,1);monitor.write(json.dumps(u)+'\n');monitor.flush()
                if max(u['cpu'],u['ram'])>=85 or time.monotonic()-start>600:
                    stop='resource threshold' if max(u['cpu'],u['ram'])>=85 else '10 minute batch limit'
                    child.terminate();child.wait(timeout=10);break
        finally:
            if child.poll() is None:child.terminate();child.wait(timeout=10)
    after=hashes();summary=dict(exit_code=child.returncode,stop_reason=stop,elapsed_s=time.monotonic()-start,sources_unchanged=before==after)
    (OUT/('guard_summary'+suffix+'.json')).write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
if __name__=='__main__':
    worker() if len(sys.argv)>1 and sys.argv[1]=='worker' else supervise()
