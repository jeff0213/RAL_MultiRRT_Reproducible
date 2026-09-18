from pathlib import Path
import pandas as pd, numpy as np
R=Path(__file__).resolve().parents[1]
f=pd.read_csv(R/'data/raw/fixed_runs.csv'); r=pd.read_csv(R/'data/raw/random_runs.csv'); df=pd.concat([f,r],ignore_index=True); df.to_csv(R/'data/raw/all_runs.csv',index=False)

def summ(g):
    v=g[g.valid==1]
    return pd.Series({'attempts':len(g),'valid':int(g.valid.sum()),'success_rate':g.valid.mean(),
      'median_length':v.mean_path_length.median(),'iqr_length':v.mean_path_length.quantile(.75)-v.mean_path_length.quantile(.25),
      'median_turning':v.turning_angle.median(),'median_clearance':v.min_clearance.median(),'median_runtime':g.runtime.median(),
      'median_collision_checks':g.collision_checks.median()})
for exp,name in [('fixed','fixed_summary.csv'),('random','random_map_summary.csv')]:
    out=df[df.experiment==exp].groupby(['map_group','algorithm'],sort=False).apply(summ,include_groups=False).reset_index()
    out.to_csv(R/'results/tables'/name,index=False)
comps=[]
for exp in ['fixed','random']:
    sub=df[df.experiment==exp]; key=['map_id','seed']
    a=sub[sub.algorithm=='APFMultiRRT'].set_index(key); b=sub[sub.algorithm=='RALMultiRRT'].set_index(key)
    for k in a.index.intersection(b.index):
        ra,rb=a.loc[k],b.loc[k]
        comps.append({'experiment':exp,'map_id':k[0],'seed':k[1],'apf_valid':ra.valid,'ral_valid':rb.valid,
        'delta_runtime':rb.runtime-ra.runtime,
        'delta_length':(rb.mean_path_length-ra.mean_path_length) if ra.valid and rb.valid else np.nan,
        'delta_turning':(rb.turning_angle-ra.turning_angle) if ra.valid and rb.valid else np.nan,
        'delta_clearance':(rb.min_clearance-ra.min_clearance) if ra.valid and rb.valid else np.nan})
pd.DataFrame(comps).to_csv(R/'data/processed/apf_vs_ral_paired.csv',index=False)
print('combined',len(df),'runs')
