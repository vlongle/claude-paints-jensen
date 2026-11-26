"""SO-101 forward kinematics and fixed-wrist-inclination inverse kinematics, in mm."""
import numpy as np

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
# XYZ in mm, roll/pitch/yaw in radians. Rounded fixed transforms from SO101 new-calib URDF.
ORIGINS = [
    ((38.8353, 0, 62.4), (np.pi, 0, -np.pi)),
    ((-30.3992, -18.2778, -54.2), (-np.pi/2, -np.pi/2, 0)),
    ((-112.57, -28, 0), (0, 0, np.pi/2)),
    ((-134.9, 5.2, 0), (0, 0, -np.pi/2)),
    ((0, -61.1, 18.1), (np.pi/2, 0.0486795, np.pi)),
]
LIMITS = np.array([[-110,110],[-100,100],[-96.8,96.8],[-95,95],[-157,162.7]])
# A nominal mounted tip in the wrist-roll link frame. Local teaching absorbs mount differences.
TIP = np.array([-10., -107.5, -137.])


def fixed(xyz, rpy):
    a,b,c = rpy
    ca,cb,cc = np.cos(rpy)
    sa,sb,sc = np.sin(rpy)
    matrix = np.eye(4)
    matrix[:3,:3] = [[cc*cb,cc*sb*sa-sc*ca,cc*sb*ca+sc*sa],
                     [sc*cb,sc*sb*sa+cc*ca,sc*sb*ca-cc*sa],[-sb,cb*sa,cb*ca]]
    matrix[:3,3] = xyz
    return matrix


BASE = [fixed(*origin) for origin in ORIGINS]


def forward(q, links=False):
    """Evaluate one or more 5-joint poses. Input angles are degrees."""
    q = np.atleast_2d(np.asarray(q,dtype=float))
    t = np.broadcast_to(np.eye(4), (len(q),4,4)).copy()
    pivots = []
    for i, origin in enumerate(BASE):
        t = t @ origin
        pivots.append(t[:,:3,3].copy())
        x,y = t[:,:,0].copy(),t[:,:,1].copy()
        angle = np.deg2rad(q[:,i,None])
        t[:,:,0] = x*np.cos(angle) + y*np.sin(angle)
        t[:,:,1] = y*np.cos(angle) - x*np.sin(angle)
    tip = t[:,:3,:3] @ TIP + t[:,:3,3]
    return np.stack([*pivots,tip],axis=1) if links else tip


def inverse(target, seed):
    """Solve XYZ with the seed's roll and shoulder+elbow+wrist pitch sum held fixed."""
    q = np.asarray(seed,dtype=float).copy()
    target = np.asarray(target,dtype=float)
    if q.shape != (len(target),5) or not np.isfinite(q).all() or not np.isfinite(target).all():
        raise ValueError('invalid starting pose or target')
    inclination = q[:,1:4].sum(axis=1)
    for _ in range(45):
        xyz = forward(q)
        err = target-xyz
        if np.max(np.linalg.norm(err,axis=1)) < .04:
            break
        jac = []
        for joint in range(3):
            trial = q.copy()
            trial[:,joint] += .02
            if joint in (1,2):
                trial[:,3] -= .02
            jac.append((forward(trial)-xyz)/.02)
        jac = np.stack(jac,axis=2)
        try:
            step = np.linalg.solve(jac,err[...,None])[...,0]
        except np.linalg.LinAlgError as e:
            raise ValueError('singular arm pose; move the working rectangle') from e
        q[:,:3] += step * np.minimum(1., 8./np.maximum(np.abs(step).max(axis=1,keepdims=True),1e-12))
        q[:,3] = inclination-q[:,1]-q[:,2]
    if np.any(np.linalg.norm(forward(q)-target,axis=1)>.15):
        raise ValueError('unreachable marker target; move or reduce the working rectangle')
    if np.any(q < LIMITS[:,0]+.5) or np.any(q > LIMITS[:,1]-.5):
        raise ValueError('planned route leaves the SO-101 joint limits')
    return q
