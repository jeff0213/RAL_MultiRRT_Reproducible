import subprocess,sys,shutil
from pathlib import Path
R=Path(__file__).resolve().parent
# Remove generated artifacts first so stale map/path combinations cannot survive a rerun.
for rel in ['data/raw','data/maps','data/paths','data/processed','results/tables','results/figures']:
    p=R/rel; p.mkdir(parents=True,exist_ok=True)
    for x in p.iterdir():
        if x.is_dir(): shutil.rmtree(x)
        else: x.unlink()
for rel in ['results/EXPERIMENT_REPORT.md','results/path_audit.json','results/geometry_QA.json']:
    p=R/rel
    if p.exists(): p.unlink()
steps=[
 [sys.executable,'tests/test_core.py'],
 [sys.executable,'tests/test_exact_geometry.py'],
 [sys.executable,'experiments/run_fixed.py'],
 [sys.executable,'experiments/run_remaining_resumable.py'],
 [sys.executable,'analysis/combine_results.py'],
 [sys.executable,'analysis/analyze.py'],
 [sys.executable,'analysis/statistics_extra.py'],
 [sys.executable,'analysis/audit_saved_paths.py'],
 [sys.executable,'analysis/plot_paths.py'],
]
for cmd in steps:
 print('RUN',' '.join(cmd),flush=True); subprocess.run(cmd,cwd=R,check=True)
print('All experiments reproduced with exact_geometry_v2 validation.')
