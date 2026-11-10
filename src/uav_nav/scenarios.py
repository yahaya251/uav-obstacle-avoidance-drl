"""Deterministic 2D fixtures showing both operation and APF limitations."""
import csv
import json
import math
from pathlib import Path

from .kinematics import KinematicEnv,Config as ToyConfig
from .core import (Config,PoseSample,ScanSample,build_observation,apf_velocity,
                   limit_velocity,MonitorWriter)

SCENARIOS = [
    ('open_route',{'start':(-4.,0.),'goal':(4.,0.),'obstacles':[]}),
    ('offset_obstacles',{'start':(-4.,-1.),'goal':(4.,1.),
                         'obstacles':[(-.5,1.5,.55),(1.5,-1.5,.55)]}),
    ('symmetric_local_minimum',{'start':(-1.8,0.),'goal':(4.,0.),
                              'obstacles':[(0.,y,.35) for y in (-1.2,-.6,0.,.6,1.2)]
                                  +[(x,y,.35) for x in (-.6,-1.2,-1.8,-2.4) for y in (-1.2,1.2)]}),
    ('stale_scan_injection',{'start':(-4.,0.),'goal':(4.,0.),'obstacles':[]}),
]


def run_scenarios(output):
    destination=Path(output)
    destination.mkdir(parents=True,exist_ok=True)
    summaries=[]
    c=Config()
    for name,options in SCENARIOS:
        env=KinematicEnv(ToyConfig(max_steps=200))
        env.reset(seed=56,options=options)
        previous=(0.,0.)
        trajectory=[]
        return_sum=0.
        injected_stops=0
        with MonitorWriter(destination/name) as writer:
            for step in range(env.cfg.max_steps):
                now=step*env.cfg.dt
                pose=PoseSample((*env.position,1.5),(*env.velocity,0.),0.,now)
                age=.7 if name=='stale_scan_injection' and 20 <= step < 25 else 0.
                raw=tuple(env.ray_distance(i*math.tau/360) for i in range(360))
                scan=ScanSample(raw,0.,math.tau/360,.01,6.,now-age)
                observation=build_observation(scan,pose,env.goal,now=now,cfg=c)
                candidate=apf_velocity(observation,c)
                action=limit_velocity(candidate,previous,env.cfg.dt,cfg=c,
                                      ready=observation.ready,min_range_m=observation.min_range_m)
                writer.write(now,pose,observation,action)
                injected_stops+=int(age>0 and action.velocity_enu==(0.,0.))
                trajectory.append({'step':step,'t_s':now,'east_m':env.position[0],
                                   'north_m':env.position[1],'goal_east_m':env.goal[0],
                                   'goal_north_m':env.goal[1],
                                   'command_east_m_s':action.velocity_enu[0],
                                   'command_north_m_s':action.velocity_enu[1],
                                   'ready':observation.ready,'action_reason':action.reason})
                _,reward,terminated,truncated,info=env.step(action.velocity_enu)
                previous=action.velocity_enu
                return_sum+=reward
                if terminated or truncated:break
        with (destination/name/'trajectory.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=trajectory[0])
            writer.writeheader()
            writer.writerows(trajectory)
        summaries.append({'scenario':name,'seed':56,'scope':'2D_engineering_fixture_not_SITL_or_research_benchmark',
                          'geometry_for_evaluation_only':options,'outcome':info,
                          'reward_sum':return_sum,'injected_stale_zero_commands':injected_stops,
                          'trajectory':f'{name}/trajectory.csv'})
    # Expected fixture behavior is part of integration acceptance, not a claim
    # that APF succeeds in arbitrary worlds.
    assert summaries[0]['outcome']['success'],'Open-route fixture failed.'
    assert summaries[2]['outcome']['timeout'],'The documented APF symmetry trap was not reproduced.'
    assert summaries[3]['injected_stale_zero_commands']==5,'Stale scan injection did not stop recommendations.'
    report={'scope':'engineering_development_only','ppo_training_performed':False,
            'formal_test_split_or_performance_estimate':False,
            'success_rate_not_reported':True,'scenarios':summaries}
    (destination/'scenario_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
