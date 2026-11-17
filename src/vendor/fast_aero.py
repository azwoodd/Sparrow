"""Vectorized evaluation of the unchanged strip-force equations; validated separately."""
import numpy as np
import mujoco
from sparrow_sim import RHO,CN_SLOPE,CN_MAX

def install(s):
    bids=np.array([x[0] for x in s.strips]); pos=np.array([x[1] for x in s.strips]);area=np.array([x[2] for x in s.strips]);normal=np.array([x[3] for x in s.strips])
    unique=np.unique(bids); index=np.searchsorted(unique,bids); vel=np.zeros((len(unique),6))
    def apply():
        s.d.qfrc_applied[:]=0;s.aero_force[:]=0
        if not s.aero_on:return
        rot=s.d.xmat[bids].reshape(-1,3,3)
        r=np.einsum('nij,nj->ni',rot,pos); n=np.einsum('nij,nj->ni',rot,normal)
        for i,b in enumerate(unique):mujoco.mj_objectVelocity(s.m,s.d,mujoco.mjtObj.mjOBJ_BODY,int(b),vel[i],0)
        v=vel[index,3:]+np.cross(vel[index,:3],r)-s.wind
        sp=np.linalg.norm(v,axis=1);vn=np.sum(v*n,axis=1)
        sa=np.abs(vn)/np.maximum(sp,1e-30)
        cn=np.minimum(CN_SLOPE*sa,np.maximum(CN_MAX,2*sa))
        f=(-.5*RHO*area*sp*sp*cn*np.where(vn>=0,1.,-1.))[:,None]*n
        f[sp<1e-6]=0
        for b in unique:
            mask=bids==b
            mujoco.mj_applyFT(s.m,s.d,f[mask].sum(axis=0),np.cross(r[mask],f[mask]).sum(axis=0),s.d.xpos[b],int(b),s.d.qfrc_applied)
        s.aero_force[:]=f.sum(axis=0)
    s.apply_aero=apply
