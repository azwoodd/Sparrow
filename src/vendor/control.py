"""Isolated Sparrow balance experiments. Original model and harness are read-only."""
import os, sys, json, csv, math, time, argparse, hashlib
from pathlib import Path
import numpy as np
SOURCE = Path(os.environ.get('SPARROW_SOURCE', r'C:\Users\Work\Documents\Codex\2026-11-11\blender-mcp-c-users-work-agents\outputs\Sparrow_V7D_Style\mujoco'))
sys.dont_write_bytecode = True
sys.path.insert(0, str(SOURCE))
import mujoco
from sparrow_sim import SparrowSim
HERE = Path(__file__).resolve().parent
OUT = HERE / 'results'
OUT.mkdir(exist_ok=True)
DIRECTIONS = {'forward':(0,-1,0),'backward':(0,1,0),'left':(1,0,0),'right':(-1,0,0)}

class Experiment:
    def __init__(self):
        self.s = SparrowSim()
        from fast_aero import install
        install(self.s)
        self.mass = self.s.m.body_mass.copy()
        self.inertia = self.s.m.body_inertia.copy()
        self.ipos = self.s.m.body_ipos.copy()
        self.friction = self.s.m.geom_friction.copy()
        self.legs = [f'leg_{side}_{part}' for side in 'LR' for part in ('thigh','shin','ankle')]
        self.ids = np.array([self.s.act['act_'+j] for j in self.legs])
        self.qi = np.array([self.s.jq[j] for j in self.legs])
        self.vi = np.array([self.s.jv[j] for j in self.legs])
        self.feet = [mujoco.mj_name2id(self.s.m,mujoco.mjtObj.mjOBJ_GEOM,'col_foot_'+side) for side in 'LR']
    def reset(self, case):
        s=self.s
        s.m.body_mass[:]=self.mass*case.get('mass',1.)
        s.m.body_inertia[:]=self.inertia*case.get('mass',1.)
        s.m.body_ipos[:]=self.ipos
        s.m.body_ipos[1,1]+=case.get('com_shift',0.)
        s.m.geom_friction[:]=self.friction
        s.m.geom_friction[:,0]*=case.get('friction',1.)
        mujoco.mj_setConst(s.m,s.d)
        s.reset_balanced(case.get('pose','stand_wings_stowed'))
        self.base=s.d.ctrl.copy()
        self.previous=self.base[self.ids].copy()
        # Initial-condition lean only, never altered during integration.
        a=math.radians(case.get('lean',0.)); axis=case.get('lean_axis','pitch')
        s.d.qpos[3:7]=[math.cos(a/2), math.sin(a/2) if axis=='pitch' else 0, math.sin(a/2) if axis=='roll' else 0, 0]
        mujoco.mj_forward(s.m,s.d)
        self.case=case
    def state(self):
        s=self.s; R=s.d.xmat[1].reshape(3,3)
        # Bird faces -Y: pitch is rotation about X, roll about Y.
        pitch=math.atan2(R[2,1],R[2,2]); roll=math.atan2(-R[2,0],R[2,2])
        vel=np.zeros(6); mujoco.mj_objectVelocity(s.m,s.d,mujoco.mjtObj.mjOBJ_BODY,1,vel,1)
        return pitch,roll,vel
    def contacts(self):
        return [int(any((c.geom1==g or c.geom2==g) and c.dist<=0 for c in self.s.d.contact)) for g in self.feet]
    def command(self, mode, gains, action=None):
        s=self.s; s.d.ctrl[:]=self.base
        if mode=='baseline': return
        p,r,v=self.state(); kp,kd,hip,lat=gains[:4]
        if len(gains)>4 and gains[4]=='asymmetric':
            if p<0:kp,kd=gains[5:7]
        u=np.clip(kp*p+kd*v[0],-.45,.45)
        lateral=np.clip(lat*(r+.12*v[1]),-.18,.18)
        delta=np.array([hip*u,lateral,-u-hip*u,hip*u,-lateral,-u-hip*u])
        if action is not None: delta += np.asarray(action)*.12
        target=self.base[self.ids]+delta
        if len(gains)>4 and gains[4]=='torque':
            target=self.base[self.ids].copy()
            for k in (2,5):
                # Outer attitude PD requests ankle torque; the existing position
                # servo supplies it, with its unchanged 0.2 Nm force clamp.
                torque=np.clip(kp*(p-.0486)+kd*v[0],-.2,.2)
                target[k]=s.d.qpos[self.qi[k]]+torque/2.5
        target=np.clip(target,s.m.actuator_ctrlrange[self.ids,0],s.m.actuator_ctrlrange[self.ids,1])
        # Provisional command slew limit 3 rad/s, 100 Hz outer loop.
        target=np.clip(target,self.previous-.03,self.previous+.03)
        s.d.ctrl[self.ids]=target; self.previous=target.copy()
    def step(self):
        s=self.s; c=self.case
        s.d.xfrc_applied[:]=0
        if 1.0 <= s.d.time < 1.15:
            s.d.xfrc_applied[1,:3]=np.array(DIRECTIONS[c.get('direction','forward')])*c.get('force',0.)
        s.step()
    def run(self,case,mode='baseline',gains=(0,0,0,0),duration=5.,trace=None):
        self.reset(case); s=self.s; rows=[]; fall=None; strict=None; effort=0.; sat=0; peak=0.; err=0.; count=0
        dt=s.m.opt.timestep
        for i in range(round(duration/dt)):
            if i%10==0: self.command(mode,gains)
            self.step()
            p,r,v=self.state(); tilt=math.acos(np.clip(s.torso_up(),-1,1)); peak=max(peak,tilt)
            if strict is None and not s.upright(): strict=float(s.d.time)
            if fall is None and (s.torso_up()<math.cos(math.radians(45)) or s.com()[2]<.09): fall=float(s.d.time)
            f=s.d.actuator_force[self.ids]; qv=s.d.qvel[self.vi]
            effort+=float(np.sum(abs(f*qv)))*dt; sat+=int(np.any(abs(f)>=.99*np.max(abs(s.m.actuator_forcerange[self.ids]),axis=1)))
            err+=p*p+r*r; count+=1
            if trace and i%20==0:
                row={'t_s':float(s.d.time),'pitch_deg':math.degrees(p),'roll_deg':math.degrees(r),'tilt_deg':math.degrees(tilt),'com_x':s.com()[0],'com_y':s.com()[1],'com_z':s.com()[2],'contact_L':self.contacts()[0],'contact_R':self.contacts()[1]}
                for k,j in enumerate(self.legs):
                    row[j+'_rad']=s.d.qpos[self.qi[k]]; row[j+'_rad_s']=qv[k]; row[j+'_Nm']=f[k]; row[j+'_target']=s.d.ctrl[self.ids[k]]
                rows.append(row)
            if fall is not None: break
        if trace and rows:
            with open(OUT/trace,'w',newline='') as fh:
                w=csv.DictWriter(fh,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
        p,r,_=self.state()
        return dict(case=case,mode=mode,gains=list(gains),fall_s=fall,strict_loss_s=strict,upright_s=fall or duration,final_upright=s.upright(),max_tilt_deg=math.degrees(peak),rms_angle_deg=math.degrees(math.sqrt(err/count)),effort_J=effort,saturation_fraction=sat/count,final_pitch_deg=math.degrees(p),final_roll_deg=math.degrees(r))

def save(name,rows):
    (OUT/(name+'.json')).write_text(json.dumps(rows,indent=2))
    flat=[{**{k:v for k,v in row.items() if k not in ('case','gains')},**row['case'],'gains':str(row['gains'])} for row in rows]
    with open(OUT/(name+'.csv'),'w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=sorted(set().union(*(r.keys() for r in flat)))); w.writeheader();w.writerows(flat)

def audit():
    e=Experiment(); s=e.s
    paths=[p for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in str(p)]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (OUT/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
    data={'mass_kg':s.total_mass,'timestep':s.m.opt.timestep,'joints':[], 'actuators':[],'feet':{str(g):s.m.geom_size[g].tolist() for g in e.feet},'sensors':s.m.nsensor}
    for j in range(s.m.njnt):
        data['joints'].append(dict(name=mujoco.mj_id2name(s.m,mujoco.mjtObj.mjOBJ_JOINT,j),axis=s.m.jnt_axis[j].tolist(),range=s.m.jnt_range[j].tolist()))
    for i in range(s.m.nu):
        data['actuators'].append(dict(name=mujoco.mj_id2name(s.m,mujoco.mjtObj.mjOBJ_ACTUATOR,i),control=s.m.actuator_ctrlrange[i].tolist(),force=s.m.actuator_forcerange[i].tolist(),force_limited=bool(s.m.actuator_forcelimited[i]),gear=s.m.actuator_gear[i].tolist()))
    (OUT/'audit.json').write_text(json.dumps(data,indent=2));print(json.dumps(data,indent=2))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('task',choices=['audit','probe','tune','benchmark']);args=ap.parse_args()
    if args.task=='audit':audit();sys.exit()
    e=Experiment(); rows=[]
    if args.task=='probe':
        for direction in DIRECTIONS:
            for force in (0,1,2,3):
                row=e.run(dict(direction=direction,force=force),trace=f'baseline_{direction}_{force}.csv');rows.append(row);print(json.dumps(row),flush=True)
    if args.task=='tune':
        for kp in (-2.,-1.,.5,1.,2.):
            for kd in (0.,.1,.3):
                for hip in (0.,-1.):
                    gains=(kp,kd,hip,0.)
                    for direction in ('forward','backward'):
                        row=e.run(dict(direction=direction,force=2.),'feedback',gains,duration=4.);rows.append(row)
                    print(gains,[(r['upright_s'],round(r['rms_angle_deg'],2)) for r in rows[-2:]],flush=True)
    if args.task=='benchmark':
        gains=tuple(json.loads((HERE/'gains.json').read_text()))
        cases=[dict(direction=d,force=f) for d in DIRECTIONS for f in (0.,.5,1.,1.5,2.,2.5,3.)]
        cases += [dict(direction=d,force=1.5,**variation) for d in DIRECTIONS for variation in ({'friction':.7},{'friction':1.2},{'mass':.95},{'mass':1.05},{'com_shift':-.003},{'com_shift':.003})]
        cases += [dict(force=0.,lean=a,lean_axis=axis) for axis in ('pitch','roll') for a in (-5.,-2.,2.,5.)]
        for k,case in enumerate(cases):
            for mode in ('baseline','feedback'):
                row=e.run(case,mode,gains,trace=f'{mode}_{k:03d}.csv');rows.append(row)
            print(k,case,[(r['upright_s'],round(r['rms_angle_deg'],2)) for r in rows[-2:]],flush=True)
    save(args.task,rows)
