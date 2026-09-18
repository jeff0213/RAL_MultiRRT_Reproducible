import sys, random, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from src.core import World, Stats, plan_path
from src.maps import fixed_maps

def run():
    w=World(fixed_maps()[0]); s=Stats()
    assert w.segment_ok((80,80),(150,80),s)
    assert not w.segment_ok((350,290),(470,290),s)
    # Algorithm identity QA: APF branch must call APF; only RAL branch must execute look-ahead trials.
    p,v,t,a=plan_path(w,'APFMultiRRT',w.start,w.goals[1],123)
    assert a.apf_calls>0 and a.lookahead_trials==0
    p,v,t,r=plan_path(w,'RALMultiRRT',w.start,w.goals[1],123)
    assert r.apf_calls>0 and r.lookahead_trials>0
    print('core tests passed; collision_checks=',s.collision_checks,'APF calls=',a.apf_calls,'RAL lookahead=',r.lookahead_trials)
if __name__=='__main__':run()
