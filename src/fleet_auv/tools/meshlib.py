"""OBJ loader -> shells (material, tri index array) + mass-property helpers (Eberly 2002 polyhedral integrals)."""
import numpy as np
from collections import defaultdict
def load(path):
    V=[];F=[];FM=[];mat=None
    for line in open(path):
        if line.startswith('v '): V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('usemtl'): mat=line.split()[1]
        elif line.startswith('f '):
            idx=[int(p.split('/')[0])-1 for p in line.split()[1:]]
            for k in range(1,len(idx)-1): F.append((idx[0],idx[k],idx[k+1])); FM.append(mat)
    V=np.array(V); F=np.array(F); FM=np.array(FM)
    par=np.arange(len(V))
    def find(a):
        while par[a]!=a: par[a]=par[par[a]]; a=par[a]
        return a
    for a,b,c in F:
        ra=find(a); par[find(b)]=ra; par[find(c)]=ra
    roots=np.array([find(f[0]) for f in F])
    sh=defaultdict(list)
    for i,r in enumerate(roots): sh[r].append(i)
    shells=[(FM[ids[0]],np.array(ids)) for ids in sh.values()]
    return V,F,shells
def props(V,F,ids):
    """volume, centroid, inertia tensor about centroid for unit density (Eberly 2002)."""
    p0,p1,p2=V[F[ids,0]],V[F[ids,1]],V[F[ids,2]]
    d=np.cross(p1-p0,p2-p0)
    def sub(w0,w1,w2):
        t0=w0+w1; f1=t0+w2; t1=w0*w0; t2=t1+w1*t0
        f2=t2+w2*f1; f3=w0*t1+w1*t2+w2*f2
        g0=f2+w0*(f1+w0); g1=f2+w1*(f1+w1); g2=f2+w2*(f1+w2)
        return f1,f2,f3,g0,g1,g2
    fx=sub(p0[:,0],p1[:,0],p2[:,0]); fy=sub(p0[:,1],p1[:,1],p2[:,1]); fz=sub(p0[:,2],p1[:,2],p2[:,2])
    I=np.zeros(10)
    I[0]=(d[:,0]*fx[0]).sum(); I[1]=(d[:,0]*fx[1]).sum(); I[2]=(d[:,1]*fy[1]).sum(); I[3]=(d[:,2]*fz[1]).sum()
    I[4]=(d[:,0]*fx[2]).sum(); I[5]=(d[:,1]*fy[2]).sum(); I[6]=(d[:,2]*fz[2]).sum()
    I[7]=(d[:,0]*(p0[:,1]*fx[3]+p1[:,1]*fx[4]+p2[:,1]*fx[5])).sum()
    I[8]=(d[:,1]*(p0[:,2]*fy[3]+p1[:,2]*fy[4]+p2[:,2]*fy[5])).sum()
    I[9]=(d[:,2]*(p0[:,0]*fz[3]+p1[:,0]*fz[4]+p2[:,0]*fz[5])).sum()
    I*=np.array([1/6,1/24,1/24,1/24,1/60,1/60,1/60,1/120,1/120,1/120])
    m=I[0]; c=I[1:4]/m
    Ixx=I[5]+I[6]-m*(c[1]**2+c[2]**2); Iyy=I[4]+I[6]-m*(c[2]**2+c[0]**2); Izz=I[4]+I[5]-m*(c[0]**2+c[1]**2)
    Ixy=-(I[7]-m*c[0]*c[1]); Iyz=-(I[8]-m*c[1]*c[2]); Ixz=-(I[9]-m*c[2]*c[0])
    return m,c,np.array([[Ixx,Ixy,Ixz],[Ixy,Iyy,Iyz],[Ixz,Iyz,Izz]])
