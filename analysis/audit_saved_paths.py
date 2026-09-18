from pathlib import Path
import json, sys, hashlib, pandas as pd
R=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(R))
from src.maps import fixed_maps
from src.core import World,path_valid,VALIDATOR_VERSION

def sh(cfg):
    return hashlib.sha256(json.dumps(cfg,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def main():
    cmap={c['name']:c for c in fixed_maps()}; rows=[]
    for p in sorted((R/'data/paths').glob('*.json')):
        rec=json.loads(p.read_text()); mid=rec.get('map_id'); cfg=cmap.get(mid); ok=True; reason=[]
        if cfg is None:ok=False;reason.append('unknown_map')
        else:
            if rec.get('scenario_hash')!=sh(cfg):ok=False;reason.append('scenario_hash')
            if rec.get('validator_version')!=VALIDATOR_VERSION:ok=False;reason.append('validator_version')
            w=World(cfg)
            paths=rec.get('paths',[])
            if len(paths)!=3:ok=False;reason.append('path_count')
            else:
                for i,path in enumerate(paths):
                    if not path_valid(w,path):ok=False;reason.append(f'goal{i+1}_invalid')
        rows.append(dict(file=p.name,map_id=mid,algorithm=rec.get('algorithm'),seed=rec.get('seed'),
                         validator_version=rec.get('validator_version'),exact_valid=int(ok),reason=';'.join(reason)))
    d=pd.DataFrame(rows); d.to_csv(R/'results/tables/saved_path_exact_audit.csv',index=False)
    report={"validator_version":VALIDATOR_VERSION,"files_checked":len(d),"files_passed":int(d.exact_valid.sum()),
            "files_failed":int((1-d.exact_valid).sum()),"status":"PASS" if len(d) and d.exact_valid.all() else "FAIL"}
    (R/'results/path_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    if report['status']!='PASS':raise SystemExit(2)
if __name__=='__main__':main()
