"""ROS-free checks for map-only Recovery integration."""
import math, sys, unittest
from pathlib import Path
PKG=Path(__file__).resolve().parents[1]/'logitle_docking'
sys.path.insert(0,str(PKG))
from map_docking import staging_pose,dock_model_pose,base_to_model_transform,accept_map_prior
from docking_recovery import retreat_distance,corridor_clear,MotionStallWatchdog,unsafe_turn_near_dock
from docking_control import final_entry_ok

class TestMapRecovery(unittest.TestCase):
    def test_staging_and_model(self):
        f=(-.2641,.3367,.1327); s=staging_pose(f,.35); m=dock_model_pose(f,.10)
        self.assertAlmostEqual(s[0],.0828,places=4)
        self.assertAlmostEqual(s[1],.3830,places=4)
        self.assertAlmostEqual(float(base_to_model_transform(s,m)[0,2]),.45,places=4)
    def test_map_prior_rejects_corner(self):
        self.assertFalse(accept_map_prior((0.,.05,0.),(0.,0.,0.),.04,7.)[0])
    def test_recovery_bounded(self):
        self.assertAlmostEqual(retreat_distance((.1,0.,0.),(0.,0.,0.),.35,.05,.12,.55),.30)
        self.assertIsNone(retreat_distance((-.3,0.,0.),(0.,0.,0.),.35,.05,.12,.55))
    def test_clearance(self):
        self.assertFalse(corridor_clear(None,.13,.07,.15))
        self.assertFalse(corridor_clear(.34,.13,.07,.15))
        self.assertTrue(corridor_clear(.40,.13,.07,.15))
    def test_slow_stall(self):
        w=MotionStallWatchdog(2.5,.01,.004)
        self.assertFalse(w.observe(0.,(0.,0.,0.),-.005))
        self.assertTrue(w.observe(2.6,(0.,0.,0.),-.005))
    def test_dock_near_turn_and_entry(self):
        self.assertTrue(unsafe_turn_near_dock((.05,0.,0.),(0.,0.,0.),math.radians(7),.3,3))
        self.assertTrue(final_entry_ok(.01,math.radians(1),.015,2))
        self.assertFalse(final_entry_ok(.03,0.,.015,2))
    def test_server_features(self):
        s=(PKG/'precision_docking_server.py').read_text()
        for token in ('RECOVERY_STOP','RECOVERY_EXIT','self._forward_clear(remaining_exit)','self.stall_watchdog.observe','self.dock_tracker.observe',"if self.p['dry_run']: v, w = 0., 0.","age > self.p['scan_stale_stop_sec']"):
            self.assertIn(token,s)
        self.assertNotIn('goal_handle.request.target_pose.pose.position.x',s)
if __name__=='__main__': unittest.main()
