import sys, math, random, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from src.core import World, VALIDATOR_VERSION

EPS=1e-9

def ref_seg_aabb(a,b,box):
    """Independent Liang-Barsky/slab reference for closed segment vs AABB."""
    x1,y1,x2,y2=box
    dx=b[0]-a[0]; dy=b[1]-a[1]
    t0,t1=0.0,1.0
    for p,q in [(-dx,a[0]-x1),(dx,x2-a[0]),(-dy,a[1]-y1),(dy,y2-a[1])]:
        if abs(p)<EPS:
            if q < -EPS: return False
        else:
            r=q/p
            if p<0:
                if r>t1+EPS:return False
                if r>t0:t0=r
            else:
                if r<t0-EPS:return False
                if r<t1:t1=r
    return t0<=t1+EPS

def ref_seg_circle(a,b,c,r):
    """Independent quadratic segment-circle intersection; touching is collision."""
    ax,ay=a; bx,by=b; cx,cy=c
    dx,dy=bx-ax,by-ay
    fx,fy=ax-cx,ay-cy
    A=dx*dx+dy*dy
    if A<EPS:
        return fx*fx+fy*fy <= r*r+EPS
    B=2*(fx*dx+fy*dy); C=fx*fx+fy*fy-r*r
    disc=B*B-4*A*C
    if disc < -EPS:return False
    disc=max(0.0,disc); s=math.sqrt(disc)
    t1=(-B-s)/(2*A); t2=(-B+s)/(2*A)
    return (-EPS<=t1<=1+EPS) or (-EPS<=t2<=1+EPS) or (C<=EPS)

def ref_rounded_rect_collision(a,b,rect,rs):
    x1,y1,x2,y2=rect
    # Minkowski rectangle = horizontal strip U vertical strip U four corner disks.
    if ref_seg_aabb(a,b,(x1-rs,y1,x2+rs,y2)): return True
    if ref_seg_aabb(a,b,(x1,y1-rs,x2,y2+rs)): return True
    for c in [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]:
        if ref_seg_circle(a,b,c,rs): return True
    return False

def ref_segment_ok(cfg,a,b):
    rs=cfg.get('safety_radius',10.0); w=cfg['width']; h=cfg['height']
    # Eroded boundary is convex. Both endpoints must be in/on interior; boundary touching allowed only
    # if still >= rs. Obstacle touching is considered collision below.
    for p in [a,b]:
        if p[0] < rs-EPS or p[0] > w-rs+EPS or p[1] < rs-EPS or p[1] > h-rs+EPS:
            return False
    for rect in cfg.get('rectangles',[]):
        if ref_rounded_rect_collision(a,b,rect,rs): return False
    for cx,cy,r in cfg.get('circles',[]):
        if ref_seg_circle(a,b,(cx,cy),r+rs): return False
    return True

def run():
    # Regression: endpoints outside but the segment clips the rounded corner/safety region.
    cfg=dict(name='corner_regression',width=400,height=300,start=[40,40],goals=[[350,250]],safety_radius=10,
             rectangles=[[150,0,195,145]],circles=[])
    w=World(cfg)
    a=(138.0,145.0); b=(150.0,157.0)
    assert w.point_ok(a) and w.point_ok(b)
    assert not w.segment_ok(a,b), 'corner-cut regression must be rejected'

    # Circle tunneling regression: a long segment crossing a circle with both endpoints free.
    cfg2=dict(name='circle_regression',width=500,height=300,start=[20,20],goals=[[480,280]],safety_radius=10,
              rectangles=[],circles=[[250,150,30]])
    w2=World(cfg2)
    assert not w2.segment_ok((50,150),(450,150))

    # 10,000 independent analytic cross-checks against an alternative reference implementation.
    rng=random.Random(20260917); mismatches=[]
    for i in range(10000):
        kind='rect' if rng.random()<0.55 else 'circle'
        cfg=dict(name=f'qa{i}',width=1000,height=800,start=[50,50],goals=[[950,750]],safety_radius=rng.uniform(3,18),rectangles=[],circles=[])
        if kind=='rect':
            x1=rng.uniform(120,750); y1=rng.uniform(90,600); ww=rng.uniform(25,150); hh=rng.uniform(25,140)
            cfg['rectangles']=[[x1,y1,min(930,x1+ww),min(730,y1+hh)]]
        else:
            cfg['circles']=[[rng.uniform(120,880),rng.uniform(90,710),rng.uniform(15,70)]]
        a=(rng.uniform(20,980),rng.uniform(20,780)); b=(rng.uniform(20,980),rng.uniform(20,780))
        got=World(cfg).segment_ok(a,b); ref=ref_segment_ok(cfg,a,b)
        if got!=ref:
            mismatches.append(dict(i=i,kind=kind,a=a,b=b,cfg=cfg,got=got,ref=ref))
            if len(mismatches)>=5: break
    assert not mismatches, f'geometry QA mismatches: {mismatches}'
    report={
        'validator_version':VALIDATOR_VERSION,
        'corner_cut_regression':'passed',
        'circle_tunneling_regression':'passed',
        'randomized_independent_cross_checks':10000,
        'mismatches':0,
        'status':'PASS'
    }
    out=ROOT/'results'/'geometry_QA.json'; out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__': run()
