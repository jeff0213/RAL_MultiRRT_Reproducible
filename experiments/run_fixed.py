import sys,json
from pathlib import Path
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT/'experiments'))
from src.maps import fixed_maps,save_maps
from src.core import VALIDATOR_VERSION
from run_all import run_one,ALGOS,make_path_record,scenario_hash

save_maps(ROOT/'scenarios'); rows=[]; saved=set()
for cfg in fixed_maps():
  for algo in ALGOS:
    for seed in range(20):
      actual_seed=1000+seed
      row,paths=run_one(cfg,algo,actual_seed)
      row.update(experiment='fixed',map_id=cfg['name'],map_group=cfg['name'],algorithm=algo,seed=actual_seed,
                 validator_version=VALIDATOR_VERSION,scenario_hash=scenario_hash(cfg))
      rows.append(row)
      key=(cfg['name'],algo)
      if key not in saved and row['valid']:
        rec=make_path_record(cfg,algo,actual_seed,paths)
        (ROOT/'data/paths'/f"{cfg['name']}__{algo}__seed{actual_seed}.json").write_text(json.dumps(rec,indent=2),encoding='utf-8')
        saved.add(key)
pd.DataFrame(rows).to_csv(ROOT/'data/raw/fixed_runs.csv',index=False)
print('fixed',len(rows),'representative path sets',len(saved))
