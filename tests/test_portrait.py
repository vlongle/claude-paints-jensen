import hashlib
import tempfile
import unittest
from io import StringIO
from pathlib import Path
from unittest.mock import patch
import numpy as np
from portrait import hardware
from portrait.model import JOINTS
from portrait.path import check_grid, selection, trajectory, virtual_grid
from portrait.sim import run


class RecordingBus:
    """Echo goals as measurements, or freeze to simulate a blocked joint."""
    def __init__(self,q,frozen=False):
        self.q=dict(zip(JOINTS,q));self.frozen=frozen
        self.enabled=False;self.writes=[]
    def sync_read(self,name,motors,**kwargs):
        return self.q.copy()
    def read(self,name,joint,**kwargs):
        return int(self.enabled) if name=='Torque_Enable' else 0
    def sync_write(self,name,values,**kwargs):
        self.writes.append(values.copy())
        if not self.frozen:self.q.update(values)
    def enable_torque(self,joints):
        self.enabled=True


class Reproduction(unittest.TestCase):
    def test_archive_and_full_replay(self):
        data=Path(__file__).resolve().parents[1]/'portrait/strokes.json'
        self.assertEqual(hashlib.sha256(data.read_bytes()).hexdigest(),
                         'a587394c46f1e867bdfaffb51ffd5747519973fd28331bb379ede13e264e05f0')
        with tempfile.TemporaryDirectory() as directory:
            run(directory)
            record=np.load(Path(directory)/'trajectory.npz')
            self.assertEqual(set(record['stroke'][record['drawing']]),set(range(271)))
            self.assertTrue(record['contact'][record['drawing']].all())
            self.assertFalse(record['contact'][0] or record['contact'][-1])
            np.testing.assert_allclose(record['joint_degrees'][0],record['joint_degrees'][-1])

    def test_grid_and_air(self):
        grid=virtual_grid()
        path=selection('orange',0,45)
        _,h,ink,_=trajectory(path,grid,air=True)
        self.assertGreaterEqual(h[:,2].min(),.6);self.assertFalse(ink.any())
        for wrong in [grid[::-1],grid[:,:,::-1],np.zeros_like(grid),np.full_like(grid,np.nan)]:
            with self.assertRaises(ValueError):check_grid(wrong)
        with self.assertRaises(ValueError):selection('black',152,2)
        with self.assertRaises(ValueError):selection('green',-1,1)

    def test_motor_driver(self):
        q,_,ink,ids=trajectory(selection('black',0,2),virtual_grid())
        bus=RecordingBus(q[0]);log=StringIO()
        with patch('portrait.hardware.time.sleep'):
            hardware.follow(bus,q,ink,ids,log)
        self.assertTrue(bus.enabled);self.assertIn('"stroke": 1',log.getvalue())
        self.assertTrue(all(set(goal)==set(JOINTS) for goal in bus.writes))
        np.testing.assert_allclose(list(bus.q.values()),q[-1],atol=.7)
        misplaced=RecordingBus(q[0]+6)
        with patch('portrait.hardware.time.sleep'),self.assertRaises(ValueError):
            hardware.follow(misplaced,q,ink,ids,StringIO())
        self.assertFalse(misplaced.writes)

    def test_stall_holds_measured_pose(self):
        q=virtual_grid()[1,0,0];bus=RecordingBus(q,frozen=True)
        import itertools
        clock=itertools.count(0,.2)
        with patch('portrait.hardware.time.sleep'),patch('portrait.hardware.time.monotonic',side_effect=lambda:next(clock)),self.assertRaisesRegex(RuntimeError,'timeout'):
            hardware.follow(bus,np.stack([q,q+4]),[False,True],[-1,0],StringIO())
        np.testing.assert_allclose(list(bus.writes[-1].values()),q)
