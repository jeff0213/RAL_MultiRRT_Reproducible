import sys, os, json, csv, math, time, itertools, hashlib
from pathlib import Path
import numpy as np, pandas as pd
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from src.core import World, plan_path, metrics, VALIDATOR_VERSION
from src.maps import fixed_maps, random_map, save_maps
ALGOS=['MultiRRT','RRTStar','RRTStarSmart','APFMultiRRT','RALMultiRRT']

def scenario_hash(cfg):
    payload=json.dumps(cfg,sort_keys=True,separators=(',',':')).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()

def make_path_record(cfg,algo,seed,paths):
    return {
        'map_id': cfg['name'],
        'scenario_hash': scenario_hash(cfg),
        'algorithm': algo,
        'seed': int(seed),
        'validator_version': VALIDATOR_VERSION,
        'paths': paths,
    }

def run_one(cfg,algo,seed,params=None,save_path=False):
    w=World(cfg); rows=[]; paths=[]; total_time=0; agg={'samples':0,'expanded_nodes':0,'collision_checks':0,'validator_calls':0,'apf_calls':0,'lookahead_trials':0}
    all_valid=True; failure=''
    for gi,g in enumerate(w.goals):
        p,v,t,s=plan_path(w,algo,w.start,g,seed*100+gi,params); total_time+=t; paths.append(p)
        if not v: all_valid=False; failure=s.failure_reason or 'invalid'
        for k in agg: agg[k]+=getattr(s,k)
    if all_valid:
        ms=[metrics(w,p) for p in paths]
        row=dict(valid=1,complete=1,collision_free=1,runtime=total_time,
                 mean_path_length=float(np.mean([x['path_length'] for x in ms])),
                 turning_angle=float(np.mean([x['turning_angle'] for x in ms])),
                 curvature_proxy=float(np.mean([x['curvature_proxy'] for x in ms])),
                 min_clearance=float(np.min([x['min_clearance'] for x in ms])),
                 mean_node_count=float(np.mean([x['node_count'] for x in ms])),failure_reason='')
    else:
        row=dict(valid=0,complete=0,collision_free=0,runtime=total_time,mean_path_length=np.nan,turning_angle=np.nan,curvature_proxy=np.nan,min_clearance=np.nan,mean_node_count=np.nan,failure_reason=failure)
    row.update(agg)
    return row,paths

def main():
    save_maps(ROOT/'scenarios')
    raw=[]
    # Fixed maps: 10 independent seeds per method-scenario => 200 path-set attempts.
    for cfg in fixed_maps():
        for algo in ALGOS:
            for seed in range(10):
                row,paths=run_one(cfg,algo,1000+seed)
                row.update(experiment='fixed',map_id=cfg['name'],map_group=cfg['name'],algorithm=algo,seed=1000+seed)
                raw.append(row)
                if seed<3 and row['valid']:
                    out=ROOT/'data/paths'/f"{cfg['name']}__{algo}__seed{1000+seed}.json"
                    out.write_text(json.dumps(paths),encoding='utf-8')
    # Independent random maps: 8 maps per difficulty group, one planner seed per map.
    for grp in ['D1','D2','D3','Narrow']:
        for mi in range(8):
            mapseed=50000+{'D1':0,'D2':1000,'D3':2000,'Narrow':3000}[grp]+mi
            cfg=random_map(mapseed,grp)
            (ROOT/'data/maps'/f"{cfg['name']}.json").write_text(json.dumps(cfg,indent=2),encoding='utf-8')
            for algo in ALGOS:
                row,paths=run_one(cfg,algo,8000+mi)
                row.update(experiment='random',map_id=cfg['name'],map_group=grp,algorithm=algo,seed=8000+mi)
                raw.append(row)
    df=pd.DataFrame(raw); df.to_csv(ROOT/'data/raw/all_runs.csv',index=False)
    # Fixed summary
    def summ(g):
        v=g[g.valid==1]
        return pd.Series({'attempts':len(g),'valid':int(g.valid.sum()),'success_rate':g.valid.mean(),
          'median_length':v.mean_path_length.median(),'iqr_length':v.mean_path_length.quantile(.75)-v.mean_path_length.quantile(.25),
          'median_turning':v.turning_angle.median(),'median_clearance':v.min_clearance.median(),'median_runtime':g.runtime.median(),
          'median_collision_checks':g.collision_checks.median()})
    fs=df[df.experiment=='fixed'].groupby(['map_group','algorithm'],sort=False).apply(summ,include_groups=False).reset_index()
    fs.to_csv(ROOT/'results/tables/fixed_summary.csv',index=False)
    rs=df[df.experiment=='random'].groupby(['map_group','algorithm'],sort=False).apply(summ,include_groups=False).reset_index()
    rs.to_csv(ROOT/'results/tables/random_map_summary.csv',index=False)
    # Paired map-level APF vs RAL differences in random suite; fixed seed-wise too.
    comps=[]
    for exp in ['fixed','random']:
      sub=df[df.experiment==exp]
      key=['map_id','seed']
      a=sub[sub.algorithm=='APFMultiRRT'].set_index(key); b=sub[sub.algorithm=='RALMultiRRT'].set_index(key)
      idx=a.index.intersection(b.index)
      for k in idx:
        ra,rb=a.loc[k],b.loc[k]
        comps.append({'experiment':exp,'map_id':k[0],'seed':k[1],'apf_valid':ra.valid,'ral_valid':rb.valid,
                      'delta_runtime':rb.runtime-ra.runtime,
                      'delta_length':(rb.mean_path_length-ra.mean_path_length) if ra.valid and rb.valid else np.nan,
                      'delta_turning':(rb.turning_angle-ra.turning_angle) if ra.valid and rb.valid else np.nan,
                      'delta_clearance':(rb.min_clearance-ra.min_clearance) if ra.valid and rb.valid else np.nan})
    pd.DataFrame(comps).to_csv(ROOT/'data/processed/apf_vs_ral_paired.csv',index=False)

    # Ablation: look-ahead settings on S2/S3, 4 seeds each.
    ab=[]
    for cfg in [fixed_maps()[1],fixed_maps()[2]]:
      for M,H in [(1,1),(2,2),(4,2),(8,3),(12,3)]:
        for seed in range(4):
          row,_=run_one(cfg,'RALMultiRRT',12000+seed,{'M':M,'H':H})
          row.update(map_id=cfg['name'],algorithm='RALMultiRRT',M=M,H=H,seed=12000+seed); ab.append(row)
    pd.DataFrame(ab).to_csv(ROOT/'data/raw/lookahead_ablation.csv',index=False)

    # Sensitivity: C and rho; 3 seeds on S2.
    sens=[]; cfg=fixed_maps()[1]
    for C,rho in itertools.product([5,9,13],[100,140,180]):
      for seed in range(3):
        row,_=run_one(cfg,'RALMultiRRT',16000+seed,{'C':C,'rho':rho})
        row.update(map_id=cfg['name'],C=C,rho=rho,seed=16000+seed); sens.append(row)
    pd.DataFrame(sens).to_csv(ROOT/'data/raw/sensitivity_C_rho.csv',index=False)
    print('WROTE',len(df),'main runs,',len(ab),'ablation,',len(sens),'sensitivity')
if __name__=='__main__': main()
