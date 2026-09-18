import random, math, json
from pathlib import Path
from collections import deque
from .core import World


def fixed_maps():
    """Higher-difficulty fixed benchmark maps aligned with the manuscript obstacle counts.
    The geometric primitives remain rectangles and circles for reproducibility, while the
    obstacle layouts form corridors, offset barriers, clutter clusters, and APF-challenging
    structures.
    """
    return [
        dict(
            name='S1', width=1000, height=800, start=[80, 80],
            goals=[[930, 120], [930, 400], [930, 700]], safety_radius=10,
            # 12 rectangles: four offset barrier columns with alternating gaps.
            rectangles=[
                [170,   0, 220, 170], [170, 290, 220, 470], [170, 610, 220, 800],
                [340, 120, 390, 280], [340, 410, 390, 580], [340, 700, 390, 800],
                [520,   0, 570, 150], [520, 280, 570, 450], [520, 570, 570, 800],
                [710, 120, 760, 250], [710, 380, 760, 530], [710, 650, 760, 800],
            ],
            # 4 circles tighten selected gates and create local decisions.
            circles=[[275, 555, 34], [455, 195, 34], [640, 605, 30], [845, 315, 32]],
            theme='standard_mixed_clutter'
        ),
        dict(
            name='S2', width=1000, height=800, start=[80, 80],
            goals=[[930, 120], [930, 400], [930, 710]], safety_radius=10,
            # 16 rectangles: structured corridor / offset screen benchmark.
            rectangles=[
                [150,   0, 195, 145], [150, 255, 195, 390], [150, 500, 195, 650], [150, 735, 195, 800],
                [310, 110, 355, 245], [310, 355, 355, 515], [310, 680, 355, 800], [310,   0, 355,  40],
                [495,   0, 540, 165], [495, 275, 540, 415], [495, 525, 540, 660], [495, 745, 540, 800],
                [700, 130, 745, 275], [700, 390, 745, 530], [700, 720, 745, 800], [700,   0, 745,  55],
            ],
            circles=[],
            theme='structured_corridors'
        ),
        dict(
            name='S3', width=1000, height=800, start=[600, 60],
            goals=[[90, 120], [120, 690], [920, 700]], safety_radius=10,
            # 10 rectangles create a U-like trap, offset corridors, and a narrow connector.
            rectangles=[
                [320, 140, 370, 500], [320, 520, 570, 565], [535, 140, 585, 540],
                [130, 255, 245, 300], [130, 470, 245, 515],
                [690, 120, 735, 245], [690, 395, 735, 510],
                [785, 255, 895, 300], [785, 470, 895, 515],
                [440, 660, 570, 700],
            ],
            # 8 circles add clutter around exits and near the U-shape openings.
            circles=[[270, 160, 26], [270, 650, 30], [430, 620, 24], [640, 610, 28],
                     [610, 300, 28], [785, 150, 24], [820, 650, 26], [170, 390, 28]],
            theme='apf_challenging_mixed'
        ),
        dict(
            name='S4', width=1000, height=800, start=[920, 80],
            goals=[[80, 120], [90, 410], [100, 700]], safety_radius=10,
            rectangles=[],
            # 21 circles: staggered dense field with multiple homotopy classes.
            circles=[
                [800, 165, 30], [760, 315, 28], [810, 515, 30], [760, 695, 26],
                [645, 120, 24], [635, 255, 28], [660, 430, 26], [620, 605, 30], [650, 735, 22],
                [500, 180, 30], [470, 355, 28], [515, 540, 30], [470, 705, 24],
                [350, 120, 26], [335, 300, 28], [360, 475, 26], [320, 655, 30],
                [190, 220, 26], [210, 410, 28], [205, 595, 24], [170, 735, 22],
            ],
            theme='dense_circular_field'
        ),
    ]


def _grid_reachable(cfg, goals=None, step=25.0):
    """Coarse grid reachability test for map generation acceptance."""
    w = World(cfg)
    pts = [tuple(cfg['start'])] + [tuple(g) for g in (goals if goals is not None else cfg['goals'])]
    xs = [i * step for i in range(int(cfg['width'] // step) + 1)]
    ys = [i * step for i in range(int(cfg['height'] // step) + 1)]

    def snap(p):
        x = min(xs, key=lambda t: abs(t - p[0]))
        y = min(ys, key=lambda t: abs(t - p[1]))
        return (x, y)

    def neighbors(node):
        x, y = node
        for dx, dy in [(step, 0), (-step, 0), (0, step), (0, -step)]:
            q = (x + dx, y + dy)
            if 0 <= q[0] <= cfg['width'] and 0 <= q[1] <= cfg['height'] and w.point_ok(q):
                yield q

    s = snap(pts[0])
    if not w.point_ok(s):
        return False
    seen = {s}
    dq = deque([s])
    while dq:
        u = dq.popleft()
        for v in neighbors(u):
            if v not in seen:
                seen.add(v)
                dq.append(v)
    return all(snap(g) in seen for g in pts[1:])


def _line_of_sight_blocked(cfg, a, b):
    w = World(cfg)
    return not w.segment_ok(tuple(a), tuple(b), resolution=12.0)


def random_map(seed, density='D1'):
    """Independent moderate-difficulty maps for statistical replication.

    Fixed S1-S4 carry the deliberately structured hard cases. D1/D2/D3 instead use
    independently scattered mixed obstacles with increasing counts (10/18/26), so the
    random suite measures generalization without turning every replicate into a maze.
    Narrow retains an explicit passage test. All maps are accepted only if all goals are
    coarsely reachable and at least two direct start-goal rays are obstructed.
    """
    rng=random.Random(seed)
    start=[70,80]; goals=[[930,100],[930,400],[930,700]]
    cfg=dict(name=f'{density}_{seed}',width=1000,height=800,start=start,goals=goals,
             safety_radius=10,rectangles=[],circles=[],theme=f'random_{density.lower()}')

    if density=='Narrow':
        # Two offset walls with generous but nontrivial safety-inflated passages.
        cfg['rectangles']=[
            [340,0,380,250],[340,390,380,800],
            [680,0,720,430],[680,570,720,800],
        ]
        cfg['circles']=[[235,300,26],[500,185,28],[520,610,30],[835,330,26],[835,650,25]]
        return cfg

    counts={'D1':10,'D2':18,'D3':26}
    n=counts[density]
    # Three small anchors create meaningful detours for the three goal directions.
    anchors=[('circle',[405,92,30]),('circle',[505,245,31]),('rect',[565,405,625,455])]
    for typ,obs in anchors:
        (cfg['circles'] if typ=='circle' else cfg['rectangles']).append(obs)

    def clear_of_terminals(cx,cy,margin):
        return all(math.hypot(cx-x,cy-y)>margin for x,y in [start]+goals)

    attempts=0
    while len(cfg['circles'])+len(cfg['rectangles'])<n and attempts<4000:
        attempts+=1
        make_rect=rng.random() < 0.42
        if make_rect:
            ww=rng.uniform(38,72); hh=rng.uniform(38,86)
            cx=rng.uniform(150,850); cy=rng.uniform(75,725)
            if not clear_of_terminals(cx,cy,95): continue
            rect=[cx-ww/2,cy-hh/2,cx+ww/2,cy+hh/2]
            ok=True
            for x1,y1,x2,y2 in cfg['rectangles']:
                if not (rect[2]+24<x1 or rect[0]-24>x2 or rect[3]+24<y1 or rect[1]-24>y2): ok=False;break
            if not ok: continue
            for ox,oy,rr in cfg['circles']:
                qx=min(max(ox,rect[0]),rect[2]); qy=min(max(oy,rect[1]),rect[3])
                if math.hypot(ox-qx,oy-qy)<rr+22:ok=False;break
            if ok:cfg['rectangles'].append(rect)
        else:
            r=rng.uniform(20,34 if density!='D3' else 31)
            cx=rng.uniform(145,855); cy=rng.uniform(70,730)
            if not clear_of_terminals(cx,cy,r+75): continue
            ok=True
            for ox,oy,rr in cfg['circles']:
                if math.hypot(cx-ox,cy-oy)<r+rr+22:ok=False;break
            if not ok: continue
            for x1,y1,x2,y2 in cfg['rectangles']:
                qx=min(max(cx,x1),x2); qy=min(max(cy,y1),y2)
                if math.hypot(cx-qx,cy-qy)<r+22:ok=False;break
            if ok:cfg['circles'].append([cx,cy,r])

    blocked=sum(_line_of_sight_blocked(cfg,start,g) for g in goals)
    if blocked<2 or not _grid_reachable(cfg,step=25.0):
        return random_map(seed+7919,density)
    return cfg

def save_maps(root):
    p = Path(root)
    p.mkdir(parents=True, exist_ok=True)
    for cfg in fixed_maps():
        (p / f"{cfg['name']}.json").write_text(json.dumps(cfg, indent=2), encoding='utf-8')
