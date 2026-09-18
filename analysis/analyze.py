import sys, math
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
PALETTE = {
    'MultiRRT': '#7d8597',
    'RRTStar': '#4895ef',
    'RRTStarSmart': '#577590',
    'APFMultiRRT': '#f4a261',
    'RALMultiRRT': '#2a9d8f',
}
DISPLAY = {
    'MultiRRT': 'MultiRRT',
    'RRTStar': 'RRT*',
    'RRTStarSmart': 'RRT*-Smart',
    'APFMultiRRT': 'APF-MultiRRT',
    'RALMultiRRT': 'RAL-MultiRRT',
}


def bootstrap_ci(x, stat=np.median, B=2000, seed=7):
    x = np.asarray([v for v in x if np.isfinite(v)], float)
    if len(x) == 0:
        return (np.nan, np.nan)
    rng = np.random.default_rng(seed)
    vals = [stat(rng.choice(x, len(x), replace=True)) for _ in range(B)]
    return tuple(np.quantile(vals, [.025, .975]))


def main():
    df = pd.read_csv(ROOT / 'data/raw/all_runs.csv')
    fixed_df = df[df.experiment == 'fixed']
    rand_df = df[df.experiment == 'random']

    # confidence intervals by fixed map and algorithm
    rows = []
    for (m, a), g in fixed_df.groupby(['map_group', 'algorithm']):
        v = g[g.valid == 1]
        lo, hi = bootstrap_ci(v.mean_path_length.values)
        rows.append(dict(
            map=m, algorithm=a, n=len(g), nvalid=int(g.valid.sum()), success_rate=g.valid.mean(),
            median_length=v.mean_path_length.median(), length_ci_low=lo, length_ci_high=hi,
            median_runtime=g.runtime.median(), median_turning=v.turning_angle.median(),
            median_clearance=v.min_clearance.median()))
    pd.DataFrame(rows).to_csv(ROOT / 'results/tables/fixed_bootstrap_ci.csv', index=False)

    # APF/RAL descriptive paired effects
    pr = pd.read_csv(ROOT / 'data/processed/apf_vs_ral_paired.csv')
    out = []
    for exp, g in pr.groupby('experiment'):
        for col in ['delta_length', 'delta_turning', 'delta_clearance', 'delta_runtime']:
            x = g[col].dropna().values
            lo, hi = bootstrap_ci(x, np.mean)
            out.append({
                'experiment': exp, 'metric': col, 'n': len(x),
                'mean_delta': np.mean(x) if len(x) else np.nan,
                'median_delta': np.median(x) if len(x) else np.nan,
                'bootstrap_mean_ci_low': lo, 'bootstrap_mean_ci_high': hi,
            })
    pd.DataFrame(out).to_csv(ROOT / 'results/tables/apf_vs_ral_effects.csv', index=False)

    # Ablation and sensitivity summaries
    ab = pd.read_csv(ROOT / 'data/raw/lookahead_ablation.csv')
    ar = []
    for (m, M, H), g in ab.groupby(['map_id', 'M', 'H']):
        v = g[g.valid == 1]
        ar.append({'map': m, 'M': M, 'H': H, 'attempts': len(g), 'valid': int(g.valid.sum()),
                   'success_rate': g.valid.mean(), 'median_length': v.mean_path_length.median(),
                   'median_runtime': g.runtime.median(), 'median_turning': v.turning_angle.median(),
                   'median_clearance': v.min_clearance.median()})
    pd.DataFrame(ar).to_csv(ROOT / 'results/tables/lookahead_ablation_summary.csv', index=False)

    se = pd.read_csv(ROOT / 'data/raw/sensitivity_C_rho.csv')
    sr = []
    for (C, rho), g in se.groupby(['C', 'rho']):
        v = g[g.valid == 1]
        sr.append({'C': C, 'rho': rho, 'attempts': len(g), 'valid': int(g.valid.sum()),
                   'success_rate': g.valid.mean(), 'median_length': v.mean_path_length.median(),
                   'median_runtime': g.runtime.median(), 'median_clearance': v.min_clearance.median()})
    pd.DataFrame(sr).to_csv(ROOT / 'results/tables/sensitivity_C_rho_summary.csv', index=False)

    # Figures
    fixed_valid = fixed_df[fixed_df.valid == 1]
    algos = ['MultiRRT', 'RRTStar', 'RRTStarSmart', 'APFMultiRRT', 'RALMultiRRT']
    labels = [DISPLAY[a] for a in algos]

    # boxplot
    fig, ax = plt.subplots(figsize=(9.5, 5.5))
    data = [fixed_valid[fixed_valid.algorithm == a].mean_path_length.dropna().values for a in algos]
    bp = ax.boxplot(data, patch_artist=True, labels=labels, showfliers=False)
    for patch, algo in zip(bp['boxes'], algos):
        patch.set_facecolor(PALETTE[algo]); patch.set_alpha(0.5); patch.set_edgecolor(PALETTE[algo])
    for median in bp['medians']:
        median.set_color('#2f3e46'); median.set_linewidth(1.6)
    ax.set_ylabel('Mean path length per 3-goal path set (m)')
    ax.tick_params(axis='x', rotation=18)
    ax.grid(axis='y', alpha=0.18)
    fig.tight_layout()
    fig.savefig(ROOT / 'results/figures' / 'fixed_path_length_boxplot.png', dpi=220)
    plt.close(fig)

    # runtime-quality scatter
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for a in algos:
        g = fixed_valid[fixed_valid.algorithm == a]
        ax.scatter(g.runtime, g.mean_path_length, s=18, alpha=.55, label=DISPLAY[a], color=PALETTE[a])
    ax.set_xlabel('Runtime per 3-goal path set (s)')
    ax.set_ylabel('Mean path length per 3-goal path set (m)')
    ax.grid(alpha=0.18)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(ROOT / 'results/figures' / 'cost_quality_scatter.png', dpi=220)
    plt.close(fig)

    # concise report with accurate counts
    fs = pd.read_csv(ROOT / 'results/tables/fixed_summary.csv')
    rs = pd.read_csv(ROOT / 'results/tables/random_map_summary.csv')
    eff = pd.read_csv(ROOT / 'results/tables/apf_vs_ral_effects.csv')
    fixed_attempts = len(fixed_df)
    random_attempts = len(rand_df)
    txt = [
        '# RAL-MultiRRT Reproducible Experiment Report', '',
        '## Scope',
        'This package rebuilds the experiments as a clean-room, fully executable 2D geometric planning benchmark. '
        'The original uploaded manuscript did not contain the planner implementation or complete fixed-map goal coordinates, '
        'so the maps in this package are newly constructed and explicitly stored. Results below are generated from the included code; '
        'no legacy numerical results are copied.', '',
        '## Main experimental design',
        f'- Five algorithms: {", ".join(DISPLAY[a] for a in algos)}.',
        f'- Four fixed maps, 20 independent planner seeds per algorithm/map ({fixed_attempts} path-set attempts).',
        f'- Four random-map groups, 15 independently generated maps per group and algorithm ({random_attempts} path-set attempts).',
        '- Every path set contains three independent start-to-goal paths.',
        '- A shared collision/boundary validator and safety radius are used for all algorithms.',
        '- Turning statistics are computed after common 15 m arc-length resampling.',
        '- Raw per-attempt counters include collision checks, validator calls, APF calls, look-ahead trials, samples, and expanded nodes.', '',
        '## Fixed-map summary', '```', fs.to_string(index=False), '```', '',
        '## Independent random-map summary', '```', rs.to_string(index=False), '```', '',
        '## APF versus RAL paired deltas',
        'Positive `delta_runtime` means RAL is slower. Positive `delta_length` means RAL produced a longer path. '
        'These are descriptive effects rather than assumptions of universal superiority.',
        '```', eff.to_string(index=False), '```', '',
        '## Reproduction',
        'Run `python reproduce_all.py`. It executes tests, regenerates experiment CSVs, tables, figures, and this report.'
    ]
    (ROOT / 'results/EXPERIMENT_REPORT.md').write_text('\n'.join(txt), encoding='utf-8')


if __name__ == '__main__':
    main()
