"""Map the archived plan into a locally taught 136 × 108 mm drawing rectangle."""
import json
from importlib.resources import files
import numpy as np
from .model import forward, inverse, LIMITS

COLORS = ('black','green','gray','orange')
SIZE = np.array([136.,108.])
LIFT = 8.
HOME = np.array([0.,0.,1.])


def strokes():
    raw = json.loads(files('portrait').joinpath('strokes.json').read_text())
    paths = [np.array(s['points'],float)*[.9,1.] for s in raw]
    all_xy = np.concatenate(paths)
    lo,hi = all_xy.min(axis=0),all_xy.max(axis=0)
    scale = np.min((SIZE-10)/(hi-lo))
    return [(s['color'],s['label'],((xy-(lo+hi)/2)*scale+SIZE/2)/SIZE)
            for s,xy in zip(raw,paths)]


def virtual_grid():
    uvh = np.array([[u,v,h] for h in (0,1) for v in (0,.5,1) for u in (0,.5,1)])
    u,v,h = uvh.T
    target = np.column_stack([285+(0.5-v)*SIZE[1],50-u*SIZE[0],-22+h*LIFT])
    return inverse(target,np.tile([0.,-22.,45.,37.,0.],(18,1))).reshape(2,3,3,5)


def check_grid(grid):
    grid = np.asarray(grid,dtype=float)
    if grid.shape!=(2,3,3,5) or not np.isfinite(grid).all():
        raise ValueError('need both heights at all nine grid nodes, with five finite angles each')
    if np.any(grid<LIMITS[:,0]+.5) or np.any(grid>LIMITS[:,1]-.5):
        raise ValueError('taught pose is outside model limits')
    xyz = forward(grid.reshape(-1,5)).reshape(2,3,3,3)
    travel = xyz[1]-xyz[0]
    if np.any(travel[...,2]<3) or np.any(travel[...,2]>16) or np.max(np.linalg.norm(travel[...,:2],axis=-1))>9:
        raise ValueError('hover should be directly above contact; re-teach the same dot at both heights')
    for axis,expected in [(2,SIZE[0]/2),(1,SIZE[1]/2)]:
        delta=np.diff(xyz,axis=axis)
        span=np.linalg.norm(delta,axis=-1)
        if np.any(abs(span-expected)>expected*.24) or np.max(abs(delta[...,2]))>14:
            raise ValueError('grid spacing or tilt is inconsistent with the template')
    # Reject folded, backwards or transposed grids; sheet front faces up.
    across=np.diff(xyz[0],axis=1)[:-1]
    down=np.diff(xyz[0],axis=0)[:,:-1]
    if np.any(np.cross(across,down)[...,2]>-1000):
        raise ValueError('grid corners out of order; follow the printed row and column labels')
    if np.ptp(grid[...,4])>28 or np.ptp(grid[...,1:4].sum(axis=-1))>30:
        raise ValueError('keep wrist roll and inclination consistent across the sheet')
    return grid


def interpolate(grid, uvh):
    u,v,h = np.asarray(uvh).T
    if not np.isfinite(uvh).all() or np.any(uvh<0) or np.any(uvh>1):
        raise ValueError('path leaves the taught volume')
    sx,sy = u*2,v*2
    ix,iy = np.minimum(sx.astype(int),1),np.minimum(sy.astype(int),1)
    fu,fv = sx-ix,sy-iy
    total = np.zeros((len(uvh),grid.shape[-1]))
    for level,wz in [(0,1-h),(1,h)]:
        for row,wy in [(0,1-fv),(1,fv)]:
            for col,wx in [(0,1-fu),(1,fu)]:
                total += grid[level,iy+row,ix+col]*(wz*wy*wx)[:,None]
    return total


def trajectory(selected,grid,air=False):
    grid=check_grid(grid)
    xyz_grid=forward(grid.reshape(-1,5)).reshape(2,3,3,3)
    pts=[HOME.copy()]; marks=[False]; ids=[-1]
    def segment(end,mark,i):
        start=pts[-1]
        distance=np.linalg.norm((end-start)*[*SIZE,LIFT])
        count=max(1,int(np.ceil(distance/2.5)))
        pts.extend(np.linspace(start,end,count+1)[1:]); marks.extend([mark]*count);ids.extend([i]*count)
    low=.6 if air else 0.
    for i,(_,_,xy) in enumerate(selected):
        segment(np.r_[xy[0],1.],False,i)
        segment(np.r_[xy[0],low],False,i)
        for point in xy[1:]:
            segment(np.r_[point,low],not air,i)
        segment(np.r_[xy[-1],1.],False,i)
    segment(HOME,False,-1)
    uvh=np.asarray(pts)
    q=inverse(interpolate(xyz_grid,uvh),interpolate(grid,uvh))
    if np.max(abs(np.diff(q,axis=0)))>12:
        raise ValueError('discontinuous joint solution; re-teach the grid')
    return q,uvh,np.array(marks),np.array(ids)


def selection(color,start,count,swatch=False):
    if color not in COLORS: raise ValueError('unknown marker')
    if swatch:
        return [(color,'registration dash',np.array([[.46,.50],[.54,.50]]))]
    stage=[s for s in strokes() if s[0]==color]
    if start<0 or count<1 or start+count>len(stage):
        raise ValueError(f'{color} has {len(stage)} strokes; start is zero-based and count must fit')
    return stage[start:start+count]
