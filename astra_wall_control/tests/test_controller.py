import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from astra_wall_control.controller import ArmModel, WholeBodyController

URDF=Path(__file__).resolve().parents[1]/'models/my/arm.urdf'

class ControlTest(unittest.TestCase):
    def test_analytic_jacobian(self):
        model=ArmModel(URDF);p=np.array([.5,.1,1.5]);R=Rotation.from_euler('xyz',[.08,-.05,.2]).as_matrix()
        q=np.array([.1,.02,.08,-.1,.2,.01,.01]);tip,_,J=model.evaluate(p,R,q)
        for k in range(9):
            pp=p.copy();RR=R.copy();qq=q.copy();h=1e-6
            if k<3:pp[k]+=h
            elif k==3:RR=Rotation.from_euler('z',h).as_matrix()@R
            else:qq[k-4]+=h
            moved=model.evaluate(pp,RR,qq)[0]
            np.testing.assert_allclose((moved-tip)/h,J[:3,k],atol=1e-6)

    def test_gravity_virtual_work(self):
        model=ArmModel(URDF);q=np.array([.1,.02,.08,-.1,.2,.01,.01]);R=np.eye(3)
        model.evaluate(np.zeros(3),R,q);gravity=model.gravity()
        def potential(q):
            model.evaluate(np.zeros(3),R,q)
            return sum(m*9.81*(model.frames[n][:3,3]+model.frames[n][:3,:3]@xyz)[2] for n,m,xyz in model.inertia)
        for k in range(7):
            a=q.copy();b=q.copy();a[k]+=1e-6;b[k]-=1e-6
            self.assertAlmostEqual((potential(a)-potential(b))/2e-6,gravity[k],places=6)

    def test_coupled_convergence_and_tilt(self):
        model=ArmModel(URDF);control=WholeBodyController(model)
        q=np.array([0.,-.05,.1,.05,0.,.01,.01]);control.posture=q[:5].copy()
        target=np.array([2.,0.,1.4]);body=target-model.evaluate(np.zeros(3),np.eye(3),q)[0]
        p=np.array([0.,0.,body[2]]);yaw=0.;errors=[]
        for i in range(1600):
            R=Rotation.from_euler('xyz',[.02*np.sin(i*.015),.02*np.cos(i*.015),yaw]).as_matrix()
            v,tip,_=control.solve(p,R,q,target,body,.02)
            p+=v[:3]*.02;yaw+=v[3]*.02;q[:5]+=v[4:]*.02
            self.assertTrue(np.all(q>=model.lower-1e-6) and np.all(q<=model.upper+1e-6))
            self.assertLessEqual(p[0],1.611)
            errors.append(np.linalg.norm(tip-target))
        self.assertLess(max(errors[-200:]),.012)
        self.assertLess(np.linalg.norm(p-body),.05)

    def test_nonfinite_rejected(self):
        model=ArmModel(URDF)
        with self.assertRaises(ValueError):
            WholeBodyController(model).solve(np.ones(3)*np.nan,np.eye(3),np.zeros(7),np.ones(3),np.ones(3),.02)

    def test_air_goal_reversal_preserves_acceleration_and_converges(self):
        model = ArmModel(URDF)
        control = WholeBodyController(model, wall_constraint=False)
        q = np.array([0., -.05, .1, .05, 0., .01, .01])
        control.posture = q[:5].copy()
        offset = model.evaluate(np.zeros(3), np.eye(3), q)[0]
        p = np.array([1.5, 0., 1.6]); yaw = 0.
        for target in (np.array([2., .4, 1.4]), np.array([-.5, -.4, 1.1])):
            body = target-offset
            for _ in range(1600):
                previous = control.previous.copy()
                R = Rotation.from_euler('z', yaw).as_matrix()
                v, tip, _ = control.solve(p, R, q, target, body, .02)
                self.assertTrue(np.all(np.abs(v[:3]-previous[:3]) <= np.array([.35, .35, .3])*.02+1e-7))
                p += v[:3]*.02; yaw += v[3]*.02; q[:5] += v[4:]*.02
            self.assertLess(np.linalg.norm(tip-target), .012)
            self.assertLess(np.linalg.norm(p-body), .05)

if __name__=='__main__':unittest.main()
