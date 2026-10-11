"""ROS-independent geometry and RGB-D perception."""
import math
import xml.etree.ElementTree as ET
import numpy as np

NAMES = ['arm_yaw_joint','arm_shoulder_joint','arm_extension_joint',
         'arm_wrist_joint','arm_roll_joint','gripper_left_joint','gripper_right_joint']
HOME = np.array([0.,-.18,0.,.18,0.,.01,.01])

def rotation(axis, angle):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    x,y,z = a; K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def transform(xyz=(0,0,0), rpy=(0,0,0)):
    T=np.eye(4); T[:3,3]=xyz
    T[:3,:3]=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
    return T

class Kinematics:
    def __init__(self, path):
        self.root=ET.parse(path).getroot(); self.joints=list(self.root.findall('joint'))
    def frames(self, q):
        values=dict(zip(NAMES,q)); frames={'aircraft_base':np.eye(4)}; pending=list(self.joints)
        while pending:
            progress=False
            for j in pending[:]:
                p=j.find('parent').get('link')
                if p not in frames: continue
                origin=j.find('origin')
                xyz=[float(v) for v in origin.get('xyz','0 0 0').split()] if origin is not None else [0]*3
                rpy=[float(v) for v in origin.get('rpy','0 0 0').split()] if origin is not None else [0]*3
                T=transform(xyz,rpy); val=values.get(j.get('name'),0.)
                if j.get('type')!='fixed':
                    axis=np.array([float(v) for v in j.find('axis').get('xyz').split()]); U=np.eye(4)
                    if j.get('type')=='prismatic': U[:3,3]=axis*val
                    else: U[:3,:3]=rotation(axis,val)
                    T=T@U
                frames[j.find('child').get('link')]=frames[p]@T
                pending.remove(j); progress=True
            if not progress: raise ValueError('Disconnected/cyclic URDF')
        return frames
    def point(self,q,link='arm_tool_link',offset=(.05,0,0)):
        T=self.frames(q)[link]; return T[:3,:3]@np.array(offset)+T[:3,3]

def red_target(bgr, depth, K, size=.025):
    import cv2
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    mask=(((hsv[:,:,0]<10)|(hsv[:,:,0]>170))&(hsv[:,:,1]>130)&(hsv[:,:,2]>65)).astype('uint8')
    count, labels, stats, centers=cv2.connectedComponentsWithStats(mask,8)
    if count<2: return None
    idx=1+np.argmax(stats[1:,cv2.CC_STAT_AREA])
    if stats[idx,cv2.CC_STAT_AREA]<6: return None
    ys,xs=np.where(labels==idx); z=depth[ys,xs]; valid=np.isfinite(z)&(z>.025)&(z<3.)
    if valid.sum()<5: return None
    # Median visible surface + half cube size along the viewing ray.
    u,v=np.median(xs[valid]),np.median(ys[valid]); d=float(np.median(z[valid]))
    ray=np.array([(u-K[2])/K[0],(v-K[5])/K[4],1.])
    return ray*d+ray/np.linalg.norm(ray)*size/2

def blue_plane(bgr, depth, K):
    import cv2
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    mask=(hsv[:,:,0]>85)&(hsv[:,:,0]<135)&(hsv[:,:,1]>90)&(hsv[:,:,2]>50)&np.isfinite(depth)&(depth>.04)&(depth<3.)
    ys,xs=np.where(mask)
    if len(xs)<100: return None
    step=max(1,len(xs)//1200); xs=xs[::step]; ys=ys[::step]; z=depth[ys,xs]
    pts=np.column_stack(((xs-K[2])*z/K[0],(ys-K[5])*z/K[4],z))
    center=pts.mean(axis=0); _,_,vh=np.linalg.svd(pts-center,full_matrices=False); n=vh[-1]
    residual=np.abs((pts-center)@n); keep=residual<.025
    if keep.mean()<.85: return None
    pts=pts[keep]; center=pts.mean(axis=0); _,_,vh=np.linalg.svd(pts-center,full_matrices=False); n=vh[-1]
    if n@center<0: n=-n
    return n,float(n@center)

