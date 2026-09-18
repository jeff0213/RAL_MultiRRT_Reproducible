import sys, json, itertools, signal, time
from pathlib import Path
import numpy as np, pandas as pd
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT/'experiments'))
from src.maps import fixed_maps, random_map
from src.core import VALIDATOR_VERSION
from run_all import run_one, ALGOS, scenario_hash

class RunTimeout(Exception): pass

def _alarm(signum, frame): raise RunTimeout()
signal.signal(signal.SIGALRM,_alarm)

def safe_run(cfg,algo,seed,params=None,limit_s=12):
    t0=time.perf_counter(); signal.setitimer(signal.ITIMER_REAL,limit_s)
    try:
        row,paths=run_one(cfg,algo,seed,params)
        return row,paths
    except RunTimeout:
        dt=time.perf_counter()-t0
        row=dict(valid=0,complete=0,collision_free=0,runtime=dt,
                 mean_path_length=np.nan,turning_angle=np.nan,curvature_proxy=np.nan,
                 min_clearance=np.nan,mean_node_count=np.nan,failure_reason='wall_time_budget',
                 samples=0,expanded_nodes=0,collision_checks=0,validator_calls=0,apf_calls=0,lookahead_trials=0)
        return row,[]
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)

def append_csv(path,row,columns=None):
    df=pd.DataFrame([row])
    write_header=not path.exists() or path.stat().st_size==0
    df.to_csv(path,mode='a',header=write_header,index=False)

def completed_keys(path,cols):
    if not path.exists() or path.stat().st_size==0:return set()
    d=pd.read_csv(path)
    return set(tuple(str(x) for x in r) for r in d[cols].itertuples(index=False,name=None))

def enrich(row,cfg):
    row['validator_version']=VALIDATOR_VERSION; row['scenario_hash']=scenario_hash(cfg); return row

def run_random():
    out=ROOT/'data/raw/random_runs.csv'; done=completed_keys(out,['map_id','algorithm','seed'])
    total=4*15*len(ALGOS); c=len(done)
    print('random resume',c,'/',total,flush=True)
    for grp in ['D1','D2','D3','Narrow']:
        for mi in range(15):
            mapseed=50000+{'D1':0,'D2':1000,'D3':2000,'Narrow':3000}[grp]+mi
            cfg=random_map(mapseed,grp)
            (ROOT/'data/maps'/f"{cfg['name']}.json").write_text(json.dumps(cfg,indent=2),encoding='utf-8')
            for algo in ALGOS:
                seed=8000+mi; key=(str(cfg['name']),str(algo),str(seed))
                if key in done: continue
                row,_=safe_run(cfg,algo,seed,limit_s=12)
                row.update(experiment='random',map_id=cfg['name'],map_group=grp,algorithm=algo,seed=seed)
                enrich(row,cfg); append_csv(out,row); done.add(key); c+=1
                if c%20==0: print(' random',c,'/',total,flush=True)

def run_ablation():
    out=ROOT/'data/raw/lookahead_ablation.csv'; done=completed_keys(out,['map_id','M','H','seed'])
    tasks=[]
    for cfg in [fixed_maps()[0],fixed_maps()[1]]:
        for M,H in [(1,1),(2,2),(4,2),(8,3),(12,3)]:
            for j in range(3):tasks.append((cfg,M,H,12000+j))
    c=len(done); print('ablation resume',c,'/',len(tasks),flush=True)
    for cfg,M,H,seed in tasks:
        key=(str(cfg['name']),str(M),str(H),str(seed))
        if key in done:continue
        row,_=safe_run(cfg,'RALMultiRRT',seed,{'M':M,'H':H,'max_iter':150},limit_s=4)
        row.update(map_id=cfg['name'],algorithm='RALMultiRRT',M=M,H=H,seed=seed); enrich(row,cfg); append_csv(out,row);done.add(key);c+=1
        if c%20==0:print(' ablation',c,'/',len(tasks),flush=True)

def run_crho():
    out=ROOT/'data/raw/sensitivity_C_rho.csv'; done=completed_keys(out,['C','rho','seed']);cfg=fixed_maps()[1]
    tasks=[(C,rho,16000+j) for C,rho in itertools.product([5,9,13],[100,140,180]) for j in range(3)]
    c=len(done); print('C/rho resume',c,'/',len(tasks),flush=True)
    for C,rho,seed in tasks:
        key=(str(C),str(rho),str(seed))
        if key in done:continue
        row,_=safe_run(cfg,'RALMultiRRT',seed,{'C':C,'rho':rho,'max_iter':180},limit_s=4)
        row.update(map_id=cfg['name'],C=C,rho=rho,seed=seed);enrich(row,cfg);append_csv(out,row);done.add(key);c+=1
        if c%18==0:print(' C/rho',c,'/',len(tasks),flush=True)

def run_weights():
    out=ROOT/'data/raw/sensitivity_weights.csv'; done=completed_keys(out,['w_turn','w_risk','seed']);cfg=fixed_maps()[1]
    tasks=[(wt,wr,19000+j) for wt,wr in itertools.product([0.25,0.55,0.9],[0.4,0.9,1.4]) for j in range(3)]
    c=len(done);print('weights resume',c,'/',len(tasks),flush=True)
    for wt,wr,seed in tasks:
        key=(str(wt),str(wr),str(seed))
        if key in done:continue
        row,_=safe_run(cfg,'RALMultiRRT',seed,{'w_turn':wt,'w_risk':wr,'max_iter':180},limit_s=4)
        row.update(map_id=cfg['name'],w_turn=wt,w_risk=wr,seed=seed);enrich(row,cfg);append_csv(out,row);done.add(key);c+=1
        if c%18==0:print(' weights',c,'/',len(tasks),flush=True)

def run_krep():
    out=ROOT/'data/raw/sensitivity_krep.csv'; done=completed_keys(out,['krep','seed']);cfg=fixed_maps()[1]
    tasks=[(k,21000+j) for k in [0.25,0.65,1.05] for j in range(4)]
    c=len(done);print('krep resume',c,'/',len(tasks),flush=True)
    for krep,seed in tasks:
        key=(str(krep),str(seed))
        if key in done:continue
        row,_=safe_run(cfg,'RALMultiRRT',seed,{'krep':krep,'max_iter':180},limit_s=4)
        row.update(map_id=cfg['name'],krep=krep,seed=seed);enrich(row,cfg);append_csv(out,row);done.add(key);c+=1
        if c%8==0:print(' krep',c,'/',len(tasks),flush=True)

if __name__=='__main__':
    run_random();run_ablation();run_crho();run_weights();run_krep();print('remaining experiments complete',flush=True)
