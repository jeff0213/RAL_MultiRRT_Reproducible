from pathlib import Path
import json, sys, hashlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
from src.maps import fixed_maps
from src.core import World, path_valid, VALIDATOR_VERSION

OBST_FILL = '#cfd8e3'
OBST_EDGE = '#5f6f82'
INFL_EDGE = '#d98f5c'
BG = '#f8fafc'
START_COLOR = '#1b7f5a'
GOAL_COLORS = ['#be4d25', '#5c80bc', '#9c6fb6']
PATH_COLORS = {
    'MultiRRT': '#7d8597',
    'RRTStar': '#4895ef',
    'RRTStarSmart': '#577590',
    'APFMultiRRT': '#f4a261',
    'RALMultiRRT': '#2a9d8f',
}
ALGO_LABELS = {
    'MultiRRT': 'MultiRRT',
    'RRTStar': 'RRT*',
    'RRTStarSmart': 'RRT*-Smart',
    'APFMultiRRT': 'APF-MultiRRT',
    'RALMultiRRT': 'RAL-MultiRRT',
}


def scenario_hash(cfg):
    payload=json.dumps(cfg,sort_keys=True,separators=(',',':')).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def stylize_axes(ax, cfg, title=None):
    ax.set_facecolor(BG)
    ax.set_xlim(0, cfg['width'])
    ax.set_ylim(0, cfg['height'])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([0, 200, 400, 600, 800, 1000])
    ax.set_yticks([0, 200, 400, 600, 800])
    ax.tick_params(labelsize=8)
    if title:
        ax.set_title(title, fontsize=11, fontweight='semibold')
    for s in ax.spines.values():
        s.set_color('#c9d2de')


def draw_world(ax, cfg, show_inflated=True):
    rs = cfg.get('safety_radius', 10)
    for x1, y1, x2, y2 in cfg.get('rectangles', []):
        if show_inflated:
            # Exact Minkowski safety envelope: rectangle dilated by a radius-rs disk.
            ax.add_patch(FancyBboxPatch((x1-rs, y1-rs), (x2-x1)+2*rs, (y2-y1)+2*rs,
                                        boxstyle=f"round,pad=0,rounding_size={rs}", fill=False,
                                        linewidth=1.0, linestyle=(0,(4,3)), edgecolor=INFL_EDGE,
                                        alpha=0.8, zorder=1))
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1,
                               facecolor=OBST_FILL, edgecolor=OBST_EDGE, linewidth=1.15,
                               joinstyle='round', zorder=2))
    for cx, cy, r in cfg.get('circles', []):
        if show_inflated:
            ax.add_patch(Circle((cx, cy), r + rs, fill=False, linewidth=1.0,
                                linestyle=(0, (4, 3)), edgecolor=INFL_EDGE, alpha=0.8, zorder=1))
        ax.add_patch(Circle((cx, cy), r, facecolor=OBST_FILL, edgecolor=OBST_EDGE, linewidth=1.15, zorder=2))
    ax.scatter([cfg['start'][0]], [cfg['start'][1]], marker='o', s=48, c=START_COLOR,
               edgecolors='white', linewidths=0.9, zorder=6)
    ax.text(cfg['start'][0] + 12, cfg['start'][1] - 14, 'Start', color=START_COLOR,
            fontsize=8, weight='semibold', zorder=7)
    for i, g in enumerate(cfg['goals']):
        ax.scatter([g[0]], [g[1]], marker='*', s=118, c=GOAL_COLORS[i % len(GOAL_COLORS)],
                   edgecolors='white', linewidths=0.8, zorder=6)
        ax.text(g[0] - 35, g[1] + 18, f'G{i+1}', color=GOAL_COLORS[i % len(GOAL_COLORS)],
                fontsize=8, weight='semibold', zorder=7)


def find_path_file(cfg_name, algo):
    matches = sorted((R / 'data/paths').glob(f"{cfg_name}__{algo}__seed*.json"))
    return matches[0] if matches else None


def load_paths(path_file, cfg, algo):
    """Load only path records that match the current scenario and exact validator.
    Stale/raw-list path files are deliberately rejected rather than silently plotted.
    """
    if not path_file:
        return None
    rec=json.loads(path_file.read_text())
    if not isinstance(rec,dict) or 'paths' not in rec:
        raise RuntimeError(f'Stale legacy path file rejected: {path_file.name}')
    if rec.get('map_id') != cfg['name']:
        raise RuntimeError(f'Map-id mismatch in {path_file.name}')
    if rec.get('scenario_hash') != scenario_hash(cfg):
        raise RuntimeError(f'Scenario-hash mismatch in {path_file.name}')
    if rec.get('algorithm') != algo:
        raise RuntimeError(f'Algorithm-label mismatch in {path_file.name}')
    if rec.get('validator_version') != VALIDATOR_VERSION:
        raise RuntimeError(f'Validator-version mismatch in {path_file.name}')
    paths=rec['paths']
    w=World(cfg)
    if len(paths)!=len(cfg['goals']):
        raise RuntimeError(f'Incomplete path set in {path_file.name}')
    for i,p in enumerate(paths):
        if not p or not path_valid(w,p):
            raise RuntimeError(f'Exact validation failed for {path_file.name}, goal {i+1}')
    return paths


def plot_overview():
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for ax, cfg in zip(axes.ravel(), fixed_maps()):
        draw_world(ax, cfg, show_inflated=True)
        title=f"{cfg['name']}  {cfg.get('theme','').replace('_',' ')}"
        stylize_axes(ax, cfg, title=title)
    legend_lines = [
        plt.Line2D([0], [0], color=INFL_EDGE, linestyle=(0, (4, 3)), lw=1.3, label='10 m safety envelope'),
        plt.Line2D([0], [0], color=OBST_EDGE, lw=6, label='Original obstacle'),
    ]
    fig.legend(handles=legend_lines, loc='lower center', ncol=2, frameon=False, fontsize=9)
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(R / 'results/figures' / 'fixed_maps_overview.png', dpi=220)
    plt.close(fig)


def plot_ral_paths():
    for cfg in fixed_maps():
        fp = find_path_file(cfg['name'], 'RALMultiRRT')
        if not fp:
            continue
        paths = load_paths(fp,cfg,'RALMultiRRT')
        fig, ax = plt.subplots(figsize=(8.8, 6.7))
        draw_world(ax, cfg, show_inflated=True)
        for i, p in enumerate(paths):
            xs = [q[0] for q in p]; ys = [q[1] for q in p]
            ax.plot(xs, ys, linewidth=2.2, color=GOAL_COLORS[i % len(GOAL_COLORS)],
                    label=f'Path to G{i+1}', zorder=5)
        stylize_axes(ax, cfg, title=f"{cfg['name']} representative RAL-MultiRRT paths")
        ax.legend(loc='upper left', fontsize=8, frameon=True, facecolor='white', edgecolor='#d6dee8')
        fig.tight_layout()
        fig.savefig(R / 'results/figures' / f"{cfg['name']}_RAL_paths.png", dpi=220)
        plt.close(fig)


def plot_algorithm_comparisons():
    algos = ['MultiRRT', 'RRTStar', 'RRTStarSmart', 'APFMultiRRT', 'RALMultiRRT']
    for cfg in fixed_maps():
        fig, axes = plt.subplots(2, 3, figsize=(13.2, 8.1))
        axes = axes.ravel()
        for ax, algo in zip(axes, algos):
            draw_world(ax, cfg, show_inflated=True)
            paths = load_paths(find_path_file(cfg['name'], algo), cfg, algo)
            if paths:
                for p in paths:
                    xs=[q[0] for q in p]; ys=[q[1] for q in p]
                    ax.plot(xs,ys,linewidth=1.9,color=PATH_COLORS[algo],alpha=0.95,zorder=5)
            stylize_axes(ax,cfg,title=ALGO_LABELS[algo])
        axes[-1].axis('off')
        handles=[plt.Line2D([0],[0],color=PATH_COLORS[a],lw=2.3,label=ALGO_LABELS[a]) for a in algos]
        handles.append(plt.Line2D([0],[0],color=INFL_EDGE,linestyle=(0,(4,3)),lw=1.3,label='10 m safety envelope'))
        axes[-1].legend(handles=handles,loc='center',frameon=False,fontsize=10)
        fig.suptitle(f"{cfg['name']} representative paths across algorithms",fontsize=13,fontweight='semibold',y=0.98)
        fig.tight_layout(rect=[0,0,1,0.97])
        fig.savefig(R/'results/figures'/f"{cfg['name']}_algorithm_comparison.png",dpi=220)
        plt.close(fig)


if __name__ == '__main__':
    plot_overview(); plot_ral_paths(); plot_algorithm_comparisons()
    print('path figures complete; all plotted paths passed exact validation')
