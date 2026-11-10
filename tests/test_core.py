"""Contract and failure-injection tests, not flight-performance benchmarks."""
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

from uav_nav import (Config,PoseSample,ScanSample,pose_from_px4,ned_to_enu,
                    enu_to_ned,build_observation,apf_velocity,limit_velocity,MonitorWriter)
from uav_nav.kinematics import KinematicEnv,Config as ToyConfig


def pose(t=10.,yaw=0.):
    return PoseSample((0.,0.,1.5),(0.,0.,0.),yaw,t)


def scan(ranges=None,t=10.,angle_min=0.,angle_increment=math.tau/360):
    return ScanSample(tuple(ranges if ranges is not None else [math.inf]*360),
                      angle_min,angle_increment,.1,6.,t)


class FrameTests(unittest.TestCase):
    def test_ned_enu_axis_and_roundtrip(self):
        self.assertEqual(ned_to_enu((2.,1.,-3.)),(1.,2.,3.))
        self.assertEqual(enu_to_ned(ned_to_enu((2.,-4.,7.))),(2.,-4.,7.))

    def test_px4_north_identity_quaternion(self):
        p=pose_from_px4((2.,1.,-3.),(.5,.2,-.1),(1.,0.,0.,0.),10.)
        self.assertAlmostEqual(p.yaw_enu,math.pi/2)
        self.assertEqual(p.position_enu,(1.,2.,3.))
        self.assertEqual(p.velocity_enu,(.2,.5,.1))

    def test_px4_east_heading_and_sign_equivalence(self):
        q=(math.sqrt(.5),0.,0.,math.sqrt(.5))
        a=pose_from_px4((0,0,0),(0,0,0),q,10.)
        b=pose_from_px4((0,0,0),(0,0,0),tuple(-v for v in q),10.)
        self.assertAlmostEqual(a.yaw_enu,0.)
        self.assertAlmostEqual(a.yaw_enu,b.yaw_enu)

    def test_reject_unknown_body_frd_and_bad_quaternion(self):
        for kwargs in ({'pose_frame':0},{'pose_frame':2},{'velocity_frame':3}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                pose_from_px4((0,0,0),(0,0,0),(1,0,0,0),10.,**kwargs)
        for q in ((0,0,0,0),(math.nan,0,0,0)):
            with self.assertRaises(ValueError):
                pose_from_px4((0,0,0),(0,0,0),q,10.)


class ScanTests(unittest.TestCase):
    def observe(self,s=None,p=None,now=10.,cfg=None,offset=0.):
        return build_observation(s or scan(),p or pose(),(12.,-12.),now=now,cfg=cfg,
                                 sensor_yaw_offset=offset)

    def test_full_scan_inf_clear_and_40d_bounds(self):
        result=self.observe()
        self.assertTrue(result.ready)
        self.assertEqual(result.values[:36],(1.,)*36)
        self.assertEqual(result.values[36:38],(1.,-1.))
        self.assertEqual(len(result.values),40)
        self.assertEqual(result.coverage_fraction,1.)

    def test_quarter_turn_rotates_front_obstacle_to_north(self):
        rays=[math.inf]*360
        rays[0]=1.2
        result=self.observe(scan(rays),pose(yaw=math.pi/2))
        self.assertTrue(result.ready)
        self.assertAlmostEqual(result.values[9],.2)
        self.assertEqual(result.values[0],1.)

    def test_sensor_extrinsic_yaw_and_negative_angle_increment(self):
        rays=[math.inf]*360
        rays[0]=2.4
        result=self.observe(scan(rays,angle_increment=-math.tau/360),offset=math.pi)
        self.assertTrue(result.ready)
        self.assertAlmostEqual(result.values[18],.4)

    def test_720_ray_full_turn_including_duplicate_endpoints(self):
        rays=[math.inf]*720
        rays[0]=1.8
        result=self.observe(scan(rays,angle_min=-math.pi,angle_increment=math.tau/719))
        self.assertTrue(result.ready)
        self.assertEqual(result.sample_count,720)
        self.assertAlmostEqual(result.values[18],.3)

    def test_producer_clocks_are_logged_without_mixing_into_reception_age(self):
        s=replace(scan(),source_timestamp=876543.2)
        p=replace(pose(),source_timestamp=1234.)
        result=self.observe(s,p)
        self.assertTrue(result.ready)
        self.assertEqual(result.scan_age_s,0.)
        self.assertEqual(result.scan_source_timestamp,876543.2)
        self.assertEqual(result.pose_source_timestamp,1234.)

    def test_min_pool_does_not_average_away_close_obstacle(self):
        rays=[6.]*360
        rays[1]=.6
        rays[3]=3.
        result=self.observe(scan(rays))
        self.assertAlmostEqual(result.values[0],.1)

    def test_nan_negative_inf_and_out_of_range_are_unknown(self):
        for invalid in (math.nan,-math.inf,-1.,0.,.05,6.1):
            rays=[math.inf]*360
            rays[0]=invalid
            result=self.observe(scan(rays))
            with self.subTest(invalid=invalid):
                self.assertFalse(result.ready)
                self.assertEqual(result.values[0],0.)
                self.assertEqual(result.invalid_samples,1)
                self.assertTrue(all(math.isfinite(v) for v in result.values))

    def test_unknown_positive_inf_semantics_stop(self):
        result=self.observe(cfg=Config(positive_inf_is_clear=False))
        self.assertFalse(result.ready)
        self.assertEqual(result.coverage_fraction,0.)

    def test_partial_forward_scan_is_not_360_degree_free_space(self):
        result=self.observe(scan([6.]*181,angle_min=-math.pi/2))
        self.assertFalse(result.ready)
        self.assertIn('incomplete_360_degree_coverage',result.reasons)
        self.assertGreater(result.coverage_fraction,.4)
        self.assertLess(result.coverage_fraction,.6)

    def test_sparse_aliasing_rejected_even_if_every_bin_has_one_ray(self):
        # Two rotations with 20-degree gaps can otherwise populate all sectors.
        result=self.observe(scan([6.]*36,angle_increment=math.radians(20)))
        self.assertFalse(result.ready)
        self.assertIn('scan_angular_gaps',result.reasons)

    def test_stale_scan_and_stale_pose_independently_block(self):
        a=self.observe(scan(t=9.4))
        b=self.observe(p=pose(t=9.4))
        self.assertIn('stale_or_future_scan',a.reasons)
        self.assertIn('stale_or_future_pose',b.reasons)
        self.assertFalse(a.ready or b.ready)

    def test_freshness_inclusive_boundary_and_future_clock(self):
        a=self.observe(scan(t=9.5),pose(t=9.5))
        b=self.observe(scan(t=10.1),pose(t=10.1))
        self.assertTrue(a.ready)
        self.assertFalse(b.ready)
        self.assertIn('stale_or_future_scan',b.reasons)

    def test_pair_skew_rejected_although_individual_samples_fresh(self):
        result=self.observe(scan(t=9.6),pose(t=9.9))
        self.assertIn('scan_pose_reception_skew',result.reasons)

    def test_tilt_gate_and_measured_speed_contract(self):
        tilted=replace(pose(),pitch_rad=math.radians(20))
        fast=replace(pose(),velocity_enu=(1.,1.,0.))
        self.assertIn('tilt_exceeds_planar_assumption',self.observe(p=tilted).reasons)
        result=self.observe(p=fast)
        self.assertFalse(result.ready)
        self.assertIn('measured_speed_exceeds_contract',result.reasons)
        self.assertTrue(all(-1 <= v <= 1 for v in result.values[36:]))

    def test_missing_scan_or_pose_cannot_be_ready(self):
        for s,p in ((None,pose()),(scan(),None)):
            result=build_observation(s,p,(1.,0.),now=10.)
            self.assertFalse(result.ready)
            self.assertEqual(len(result.values),40)
            self.assertTrue(all(math.isfinite(v) for v in result.values))


class ActionAndLoggingTests(unittest.TestCase):
    def test_baseline_uses_only_same_observation_and_rotates_goal(self):
        a=build_observation(scan(),pose(),(5.,0.),now=10.)
        b=build_observation(scan(),pose(),(0.,5.),now=10.)
        self.assertEqual(apf_velocity(a),(1.,0.))
        self.assertEqual(apf_velocity(b),(0.,1.))
        self.assertEqual(apf_velocity(a),apf_velocity(replace(a)))

    def test_component_and_vector_norm_limits(self):
        action=limit_velocity((9.,9.),(0.,0.),10.)
        self.assertAlmostEqual(math.hypot(*action.velocity_enu),1.)
        self.assertAlmostEqual(action.velocity_enu[0],math.sqrt(.5))
        self.assertTrue(action.limited)

    def test_acceleration_norm_and_ned_boundary(self):
        action=limit_velocity((1.,0.),(0.,0.),.1)
        self.assertAlmostEqual(math.hypot(*action.velocity_enu),.1)
        self.assertEqual(action.velocity_ned,(0.,.1,-0.))

    def test_invalid_or_stale_inference_immediate_stop(self):
        for request,kwargs in (((math.nan,0.),{}),((1.,0.),{'ready':False}),
                               ((1.,0.),{'min_range_m':math.nan})):
            action=limit_velocity(request,(1.,0.),.1,**kwargs)
            self.assertEqual(action.velocity_enu,(0.,0.))
            self.assertNotEqual(action.reason,'ok')

    def test_minimum_distance_stop_boundary(self):
        c=Config()
        threshold=c.vehicle_radius+c.stop_margin
        a=limit_velocity((1.,0.),(1.,0.),.1,min_range_m=threshold)
        b=limit_velocity((1.,0.),(1.,0.),.1,min_range_m=threshold+.001)
        self.assertEqual(a.reason,'minimum_range_stop')
        self.assertEqual(a.velocity_enu,(0.,0.))
        self.assertEqual(b.velocity_enu,(1.,0.))

    def test_goal_arrival_zero_recommendation(self):
        observation=build_observation(scan(),pose(),(.1,.1),now=10.)
        self.assertEqual(apf_velocity(observation),(0.,0.))

    def test_log_schema_is_finite_and_never_actuation(self):
        obs=build_observation(scan(),pose(),(4.,0.),now=10.)
        action=limit_velocity(apf_velocity(obs),(0.,0.),.1,min_range_m=obs.min_range_m)
        with tempfile.TemporaryDirectory() as tmp:
            with MonitorWriter(tmp) as monitor:
                row=monitor.write(10.,pose(),obs,action)
            saved=json.loads((Path(tmp)/'monitor.jsonl').read_text())
            summary=json.loads((Path(tmp)/'monitor_summary.json').read_text())
            self.assertEqual(len(saved['observation']),40)
            self.assertFalse(row['flight_command_sent'])
            self.assertEqual(summary['samples'],1)
            self.assertIn('recommendation_only',saved['scope'])

    def test_invalid_pose_can_be_logged_as_rejected_without_nan_json(self):
        bad=replace(pose(),position_enu=(math.nan,0.,0.),velocity_enu=(math.nan,0.,0.))
        obs=build_observation(scan(),bad,(4.,0.),now=10.)
        self.assertFalse(obs.ready)
        action=limit_velocity((1.,0.),(0.,0.),.1,ready=obs.ready)
        with tempfile.TemporaryDirectory() as tmp:
            with MonitorWriter(tmp) as monitor:
                monitor.write(10.,bad,obs,action)
            saved=json.loads((Path(tmp)/'monitor.jsonl').read_text())
            self.assertIsNone(saved['east_m'])
            self.assertIsNone(saved['speed_m_s'])
            self.assertFalse(saved['ready'])

    def test_configuration_rejects_contract_dimension_or_negative_threshold(self):
        for kwargs in ({'rays':18},{'sensor_range':-1},{'max_scan_age':math.nan}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):Config(**kwargs)


class SimulationContractTests(unittest.TestCase):
    def test_seeded_reset_and_action_rollout_are_reproducible(self):
        a,b=KinematicEnv(),KinematicEnv()
        self.assertEqual(a.reset(seed=572),b.reset(seed=572))
        for _ in range(10):
            self.assertEqual(a.step((.1,.2)),b.step((.1,.2)))

    def test_swept_collision_prevents_tunneling(self):
        env=KinematicEnv(ToyConfig(dt=1.))
        env.reset(options={'start':(0.,0.),'goal':(4.,0.),'obstacles':[(.6,0.,.1)]})
        _,_,terminated,truncated,info=env.step((1.,0.))
        self.assertTrue(terminated and info['collision'])
        self.assertFalse(truncated)

    def test_collision_tangent_boundary(self):
        env=KinematicEnv()
        env.reset(options={'start':(0.,0.),'goal':(4.,0.),'obstacles':[(.05,.3,.1)]})
        _,_,terminated,_,info=env.step((1.,0.))
        self.assertTrue(terminated and info['collision'])

    def test_timeout_requires_explicit_reset(self):
        env=KinematicEnv(ToyConfig(max_steps=1))
        env.reset()
        _,_,term,truncated,info=env.step((0.,0.))
        self.assertTrue(truncated and info['timeout'])
        self.assertFalse(term)
        with self.assertRaises(RuntimeError):env.step((0.,0.))
        self.assertEqual(len(env.reset(seed=1)[0]),40)


if __name__ == '__main__':
    unittest.main(verbosity=2)
