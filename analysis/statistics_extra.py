from pathlib import Path
import numpy as np, pandas as pd
R=Path(__file__).resolve().parents[1]
df=pd.read_csv(R/'data/raw/all_runs.csv')
rng=np.random.default_rng(20260917)

def perm_p(d,B=10000):
    d=np.asarray(d,float); d=d[np.isfinite(d)]
    if len(d)<3:return np.nan
    obs=abs(d.mean()); cnt=0
    for _ in range(B):
        val=abs((d*rng.choice([-1,1],size=len(d))).mean())
        cnt += val>=obs-1e-15
    return (cnt+1)/(B+1)

def holm(pvals):
    p=np.array(pvals,float); out=np.full(len(p),np.nan); ok=np.where(np.isfinite(p))[0]
    if not len(ok): return out
    order=ok[np.argsort(p[ok])]; m=len(order); running=0
    for rank,i in enumerate(order):
        adj=(m-rank)*p[i]; running=max(running,adj); out[i]=min(1,running)
    return out

rows=[]
metrics=['mean_path_length','turning_angle','min_clearance','runtime']
for exp in ['fixed','random']:
    sub=df[df.experiment==exp]
    groups=sorted(sub.map_group.unique())
    for grp in groups:
      sg=sub[sub.map_group==grp]
      ral=sg[sg.algorithm=='RALMultiRRT'].set_index(['map_id','seed'])
      for algo in ['MultiRRT','RRTStar','RRTStarSmart','APFMultiRRT']:
        oth=sg[sg.algorithm==algo].set_index(['map_id','seed']); idx=ral.index.intersection(oth.index)
        for metric in metrics:
          vals=[]
          for k in idx:
            a=ral.loc[k]; b=oth.loc[k]
            if metric!='runtime' and not (a.valid and b.valid): continue
            va=float(a[metric]); vb=float(b[metric])
            if np.isfinite(va) and np.isfinite(vb): vals.append(va-vb)
          rows.append(dict(experiment=exp,map_group=grp,comparison=f'RAL-{algo}',metric=metric,n_pairs=len(vals),mean_delta=np.mean(vals) if vals else np.nan,median_delta=np.median(vals) if vals else np.nan,p_value=perm_p(vals)))
out=pd.DataFrame(rows)
out['p_holm']=np.nan
for key,g in out.groupby(['experiment','map_group','comparison']):
    out.loc[g.index,'p_holm']=holm(g.p_value.values)
out.to_csv(R/'results/tables/pairwise_permutation_tests.csv',index=False)

# extra sensitivities
for src,group_cols,name in [
 ('sensitivity_weights.csv',['w_turn','w_risk'],'sensitivity_weights_summary.csv'),
 ('sensitivity_krep.csv',['krep'],'sensitivity_krep_summary.csv')]:
    d=pd.read_csv(R/'data/raw'/src); rr=[]
    for key,g in d.groupby(group_cols):
        if not isinstance(key,tuple): key=(key,)
        v=g[g.valid==1]; row={c:x for c,x in zip(group_cols,key)}
        row.update(attempts=len(g),valid=int(g.valid.sum()),success_rate=g.valid.mean(),median_length=v.mean_path_length.median(),median_turning=v.turning_angle.median(),median_clearance=v.min_clearance.median(),median_runtime=g.runtime.median())
        rr.append(row)
    pd.DataFrame(rr).to_csv(R/'results/tables'/name,index=False)
print('extra statistics complete')
