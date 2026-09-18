import math, time, heapq, random
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any
import numpy as np

Point=Tuple[float,float]

@dataclass
class Stats:
    samples:int=0
    expanded_nodes:int=0
    collision_checks:int=0
    validator_calls:int=0
    apf_calls:int=0
    lookahead_trials:int=0
    failure_reason:str=''

VALIDATOR_VERSION = 'exact_geometry_v2'
GEOM_EPS = 1e-9


def _point_segment_distance(p:Point, a:Point, b:Point) -> float:
    """Exact Euclidean distance from point p to closed segment ab."""
    px,py=p; ax,ay=a; bx,by=b
    vx,vy=bx-ax,by-ay
    vv=vx*vx+vy*vy
    if vv <= GEOM_EPS:
        return math.hypot(px-ax,py-ay)
    t=((px-ax)*vx+(py-ay)*vy)/vv
    t=max(0.0,min(1.0,t))
    qx,qy=ax+t*vx,ay+t*vy
    return math.hypot(px-qx,py-qy)


def _orient(a:Point,b:Point,c:Point) -> float:
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _on_segment(a:Point,b:Point,p:Point) -> bool:
    return (min(a[0],b[0])-GEOM_EPS <= p[0] <= max(a[0],b[0])+GEOM_EPS and
            min(a[1],b[1])-GEOM_EPS <= p[1] <= max(a[1],b[1])+GEOM_EPS and
            abs(_orient(a,b,p)) <= GEOM_EPS)


def _segments_intersect(a:Point,b:Point,c:Point,d:Point) -> bool:
    """Closed-segment intersection; touching counts as intersection."""
    o1,o2,o3,o4=_orient(a,b,c),_orient(a,b,d),_orient(c,d,a),_orient(c,d,b)
    if ((o1 > GEOM_EPS and o2 < -GEOM_EPS) or (o1 < -GEOM_EPS and o2 > GEOM_EPS)) and \
       ((o3 > GEOM_EPS and o4 < -GEOM_EPS) or (o3 < -GEOM_EPS and o4 > GEOM_EPS)):
        return True
    if abs(o1)<=GEOM_EPS and _on_segment(a,b,c): return True
    if abs(o2)<=GEOM_EPS and _on_segment(a,b,d): return True
    if abs(o3)<=GEOM_EPS and _on_segment(c,d,a): return True
    if abs(o4)<=GEOM_EPS and _on_segment(c,d,b): return True
    return False


def _segment_segment_distance(a:Point,b:Point,c:Point,d:Point) -> float:
    if _segments_intersect(a,b,c,d):
        return 0.0
    return min(_point_segment_distance(a,c,d), _point_segment_distance(b,c,d),
               _point_segment_distance(c,a,b), _point_segment_distance(d,a,b))


def _point_rect_distance(p:Point, rect) -> float:
    """Distance to a closed axis-aligned rectangle; 0 for points inside/on it."""
    x,y=p; x1,y1,x2,y2=rect
    dx=max(x1-x,0.0,x-x2); dy=max(y1-y,0.0,y-y2)
    return math.hypot(dx,dy)


def _segment_intersects_rect(a:Point,b:Point,rect) -> bool:
    """Exact closed segment vs. axis-aligned rectangle intersection."""
    x1,y1,x2,y2=rect
    if x1-GEOM_EPS <= a[0] <= x2+GEOM_EPS and y1-GEOM_EPS <= a[1] <= y2+GEOM_EPS: return True
    if x1-GEOM_EPS <= b[0] <= x2+GEOM_EPS and y1-GEOM_EPS <= b[1] <= y2+GEOM_EPS: return True
    edges=[((x1,y1),(x2,y1)),((x2,y1),(x2,y2)),((x2,y2),(x1,y2)),((x1,y2),(x1,y1))]
    return any(_segments_intersect(a,b,c,d) for c,d in edges)




def _segment_intersects_aabb_slab(a:Point,b:Point,box) -> bool:
    """Fast exact closed-segment/AABB intersection using the slab formulation."""
    x1,y1,x2,y2=box; dx=b[0]-a[0]; dy=b[1]-a[1]
    t0,t1=0.0,1.0
    for p,q in [(-dx,a[0]-x1),(dx,x2-a[0]),(-dy,a[1]-y1),(dy,y2-a[1])]:
        if abs(p)<=GEOM_EPS:
            if q < -GEOM_EPS: return False
        else:
            r=q/p
            if p<0:
                if r>t1+GEOM_EPS:return False
                if r>t0:t0=r
            else:
                if r<t0-GEOM_EPS:return False
                if r<t1:t1=r
    return t0<=t1+GEOM_EPS


def _segment_hits_inflated_rect(a:Point,b:Point,rect,rs:float) -> bool:
    """Exact segment collision with rectangle Minkowski-summed by a radius-rs disk."""
    x1,y1,x2,y2=rect
    # Orthogonal strips plus four corner disks exactly form the rounded rectangle.
    if _segment_intersects_aabb_slab(a,b,(x1-rs,y1,x2+rs,y2)): return True
    if _segment_intersects_aabb_slab(a,b,(x1,y1-rs,x2,y2+rs)): return True
    rr=rs+GEOM_EPS
    for c in ((x1,y1),(x2,y1),(x2,y2),(x1,y2)):
        if _point_segment_distance(c,a,b) <= rr: return True
    return False

def _segment_rect_distance(a:Point,b:Point,rect) -> float:
    """Exact minimum distance between a segment and a closed axis-aligned rectangle."""
    if _segment_intersects_rect(a,b,rect):
        return 0.0
    x1,y1,x2,y2=rect
    edges=[((x1,y1),(x2,y1)),((x2,y1),(x2,y2)),((x2,y2),(x1,y2)),((x1,y2),(x1,y1))]
    return min(_segment_segment_distance(a,b,c,d) for c,d in edges)


class World:
    def __init__(self, cfg:Dict[str,Any]):
        self.name=cfg['name']; self.w=float(cfg['width']); self.h=float(cfg['height'])
        self.start=tuple(map(float,cfg['start'])); self.goals=[tuple(map(float,g)) for g in cfg['goals']]
        self.rects=[tuple(map(float,r)) for r in cfg.get('rectangles',[])]
        self.circles=[tuple(map(float,c)) for c in cfg.get('circles',[])]
        self.rs=float(cfg.get('safety_radius',10.0))

    def point_ok(self,p:Point):
        """Point feasibility against the exact safety-radius Minkowski geometry."""
        x,y=p
        # Map boundary is eroded by safety radius.
        if x < self.rs-GEOM_EPS or x > self.w-self.rs+GEOM_EPS or y < self.rs-GEOM_EPS or y > self.h-self.rs+GEOM_EPS:
            return False
        # Rectangle + disk inflation is a rounded rectangle. Distance <= rs is collision/touching.
        for rect in self.rects:
            if _point_rect_distance(p,rect) <= self.rs+GEOM_EPS:
                return False
        for cx,cy,r in self.circles:
            if math.hypot(x-cx,y-cy) <= r+self.rs+GEOM_EPS:
                return False
        return True

    def clearance(self,p:Point):
        """Signed clearance to the safety-inflated geometry (positive in free space)."""
        x,y=p; best=min(x-self.rs,y-self.rs,(self.w-self.rs)-x,(self.h-self.rs)-y)
        for x1,y1,x2,y2 in self.rects:
            outside=_point_rect_distance(p,(x1,y1,x2,y2))
            if x1 <= x <= x2 and y1 <= y <= y2:
                outside=-min(x-x1,x2-x,y-y1,y2-y)
            best=min(best,outside-self.rs)
        for cx,cy,r in self.circles:
            best=min(best,math.hypot(x-cx,y-cy)-r-self.rs)
        return best

    def segment_ok(self,a:Point,b:Point, stats:Stats=None, resolution=None):
        """Exact continuous collision check for a closed segment.

        No spatial sampling is used. Tangency to an inflated obstacle or eroded map boundary
        is treated as collision, preventing corner-cutting/tunneling at any segment length.
        The optional resolution argument is accepted only for backward compatibility and ignored.
        """
        if stats: stats.collision_checks+=1
        a=(float(a[0]),float(a[1])); b=(float(b[0]),float(b[1]))
        if not self.point_ok(a) or not self.point_ok(b):
            return False
        # The eroded map rectangle is convex, so feasible endpoints imply the entire segment
        # remains within the eroded boundary.
        for rect in self.rects:
            if _segment_hits_inflated_rect(a,b,rect,self.rs):
                return False
        for cx,cy,r in self.circles:
            if _point_segment_distance((cx,cy),a,b) <= r+self.rs+GEOM_EPS:
                return False
        return True

def turn_angle(a,b,c):
    u=np.array(a)-np.array(b); v=np.array(c)-np.array(b)
    nu=np.linalg.norm(u); nv=np.linalg.norm(v)
    if nu<1e-9 or nv<1e-9: return 0.0
    z=float(np.clip(np.dot(u,v)/(nu*nv),-1,1))
    # deflection: straight = 0
    return math.pi-math.acos(z)

def path_valid(world,path,stats=None,theta_max=math.radians(100)):
    if stats: stats.validator_calls+=1
    if not path or len(path)<2: return False
    for p in path:
        if not world.point_ok(p): return False
    for a,b in zip(path[:-1],path[1:]):
        if not world.segment_ok(a,b,stats): return False
    for i in range(1,len(path)-1):
        if turn_angle(path[i-1],path[i],path[i+1])>theta_max: return False
    return True

def shortcut(world,path,rng,stats,trials=80,theta_max=math.radians(100)):
    p=list(path)
    if len(p)<3:return p
    for _ in range(trials):
        if len(p)<3: break
        i,j=sorted(rng.sample(range(len(p)),2))
        if j<=i+1: continue
        q=p[:i+1]+p[j:]
        if world.segment_ok(p[i],p[j],stats) and path_valid(world,q,stats,theta_max): p=q
    return p

def densify(path,spacing=12.0):
    out=[path[0]]
    for a,b in zip(path[:-1],path[1:]):
        d=math.dist(a,b); n=max(1,int(math.ceil(d/spacing)))
        for k in range(1,n+1):
            t=k/n; out.append((a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t))
    return out

def safe_refine(world,path,rng,stats):
    if not path:return path
    q=shortcut(world,path,rng,stats,80)
    return densify(q,12.0)

def reconstruct(nodes,parents,idx):
    p=[]; seen=set()
    while idx!=-1:
        if idx in seen:
            raise RuntimeError('parent-cycle detected in planner tree')
        seen.add(idx); p.append(nodes[idx]); idx=parents[idx]
    return list(reversed(p))

def nearest(nodes,p):
    arr=np.asarray(nodes); d=((arr[:,0]-p[0])**2+(arr[:,1]-p[1])**2)
    return int(np.argmin(d))

def steer(a,b,step):
    d=math.dist(a,b)
    if d<=step:return b
    t=step/d; return (a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t)

def rrt_plan(world,start,goal,rng,stats,star=False,smart=False,max_iter=260,step=60.0,goal_bias=.35):
    nodes=[start]; parents=[-1]; costs=[0.0]; children=[set()]; goal_idx=None; found_iter=None

    def is_ancestor(anc,node):
        cur=node
        while cur!=-1:
            if cur==anc:return True
            cur=parents[cur]
        return False

    def refresh_subtree(root):
        stack=[root]
        while stack:
            u=stack.pop()
            for ch in children[u]:
                costs[ch]=costs[u]+math.dist(nodes[u],nodes[ch])
                stack.append(ch)

    for it in range(max_iter):
        stats.samples+=1
        if rng.random()<goal_bias: samp=goal
        else: samp=(rng.uniform(world.rs,world.w-world.rs),rng.uniform(world.rs,world.h-world.rs))
        if not world.point_ok(samp): continue
        ni=nearest(nodes,samp); new=steer(nodes[ni],samp,step)
        if not world.point_ok(new) or not world.segment_ok(nodes[ni],new,stats): continue
        par=ni; newcost=costs[ni]+math.dist(nodes[ni],new)
        near=[]
        if star:
            rad=70.0
            near=[j for j,n in enumerate(nodes) if math.dist(n,new)<=rad]
            for j in near:
                c=costs[j]+math.dist(nodes[j],new)
                if c<newcost and world.segment_ok(nodes[j],new,stats): par=j; newcost=c
        nodes.append(new); parents.append(par); costs.append(newcost); children.append(set())
        idx=len(nodes)-1; children[par].add(idx); stats.expanded_nodes+=1
        if star:
            for j in near:
                if j==0 or j==par or is_ancestor(j,idx):
                    continue
                c=newcost+math.dist(new,nodes[j])
                if c+1e-9<costs[j] and world.segment_ok(new,nodes[j],stats):
                    old=parents[j]
                    if old>=0: children[old].discard(j)
                    parents[j]=idx; children[idx].add(j); costs[j]=c
                    refresh_subtree(j)
        if math.dist(new,goal)<=step and world.segment_ok(new,goal,stats):
            nodes.append(goal); parents.append(idx); costs.append(newcost+math.dist(new,goal)); children.append(set()); children[idx].add(len(nodes)-1)
            goal_idx=len(nodes)-1
            if found_iter is None: found_iter=it
            if not star or it-found_iter>20: break
    if goal_idx is None: stats.failure_reason='no_path'; return None
    p=reconstruct(nodes,parents,goal_idx)
    if smart: p=shortcut(world,p,rng,stats,120)
    return safe_refine(world,p,rng,stats)

def apf_direction(world,p,goal,stats,katt=1.0,krep=.65,rho=140.0):
    stats.apf_calls+=1
    g=np.array(goal)-np.array(p); ng=np.linalg.norm(g)
    if ng<1e-9:return 0.0
    F=katt*g/ng
    # numeric gradient of clearance-like obstacle repulsion via primitives
    eps=2.0
    # Use local clearance gradient away from obstacles/bounds
    c0=world.clearance(p)
    if c0<rho:
        cx1=world.clearance((min(world.w-world.rs,p[0]+eps),p[1])); cx0=world.clearance((max(world.rs,p[0]-eps),p[1]))
        cy1=world.clearance((p[0],min(world.h-world.rs,p[1]+eps))); cy0=world.clearance((p[0],max(world.rs,p[1]-eps)))
        grad=np.array([(cx1-cx0)/(2*eps),(cy1-cy0)/(2*eps)])
        ngd=np.linalg.norm(grad)
        if ngd>1e-9:
            strength=krep*max(0.0,(1/max(c0,1e-3)-1/rho))*min(25.0,1/max(c0,1e-3))
            F=F+strength*grad/ngd
    if np.linalg.norm(F)<1e-9:return math.atan2(g[1],g[0])
    return math.atan2(F[1],F[0])

def candidate_set(world,p,goal,prev,rng,stats,C=9,step=45.0,rho=140.0,krep=.65):
    th=apf_direction(world,p,goal,stats,rho=rho,krep=krep)
    angles=np.linspace(math.radians(-55),math.radians(55),C)
    out=[]
    for da in angles:
        a=th+float(da); q=(p[0]+step*math.cos(a),p[1]+step*math.sin(a)); stats.samples+=1
        if not world.point_ok(q) or not world.segment_ok(p,q,stats): continue
        if prev is not None and turn_angle(prev,p,q)>math.radians(100): continue
        out.append(q)
    # deterministic goal candidate when close
    if math.dist(p,goal)<=step and world.segment_ok(p,goal,stats) and (prev is None or turn_angle(prev,p,goal)<=math.radians(100)+GEOM_EPS): out.append(goal)
    # APF-centered exploratory supplements: preserve the APF family but avoid brittle dead-ends.
    # When the local fan is sparse, add a few wider-angle feasible options sharing the same step budget.
    if len(out) < max(3, C // 3):
        for da in np.linspace(-math.pi, math.pi, 12, endpoint=False):
            a=th+float(da); q=(p[0]+step*math.cos(a),p[1]+step*math.sin(a)); stats.samples+=1
            if not world.point_ok(q) or not world.segment_ok(p,q,stats): continue
            if prev is not None and turn_angle(prev,p,q)>math.radians(100): continue
            if all(math.dist(q,z) > step*0.25 for z in out):
                out.append(q)
    if not out:
        for da in np.linspace(-math.pi, math.pi, 16, endpoint=False):
            a=th+float(da); q=(p[0]+step*math.cos(a),p[1]+step*math.sin(a)); stats.samples+=1
            if not world.point_ok(q) or not world.segment_ok(p,q,stats): continue
            if prev is not None and turn_angle(prev,p,q)>math.radians(100): continue
            out.append(q)
    return out

def local_score(world,p,q,goal,prev,rho=140.0,w_turn=.55,w_risk=.9):
    D=math.dist(q,goal)/max(world.w,world.h)
    L=math.dist(p,q)/max(world.w,world.h)
    T=(turn_angle(prev,p,q)/math.pi if prev else 0.0)
    R=max(0.0,(rho-max(world.clearance(q),0.0))/rho)
    return 1.0*D+0.35*L+w_turn*T+w_risk*R

def rollout_score(world,p,q,goal,prev,rng,stats,M=8,H=3,step=45.0,rho=140.0,krep=.65,w_turn=.55,w_risk=.9):
    scores=[]
    for _ in range(M):
        stats.lookahead_trials+=1
        cur=q; prv=p; total=local_score(world,p,q,goal,prev,rho,w_turn,w_risk)
        ok=True
        for h in range(H):
            if math.dist(cur,goal)<=step:
                if world.segment_ok(cur,goal,stats): total += 0.1*math.dist(cur,goal)/max(world.w,world.h); cur=goal
                break
            th=apf_direction(world,cur,goal,stats,rho=rho,krep=krep)+rng.uniform(-.35,.35)
            nxt=(cur[0]+step*math.cos(th),cur[1]+step*math.sin(th))
            if not world.point_ok(nxt) or not world.segment_ok(cur,nxt,stats) or turn_angle(prv,cur,nxt)>math.radians(100): ok=False; break
            total += .55*local_score(world,cur,nxt,goal,prv,rho,w_turn,w_risk); prv,cur=cur,nxt
        if ok:scores.append(total)
        else:scores.append(total+0.8)
    return float(np.mean(scores))

def apf_plan(world,start,goal,rng,stats,lookahead=False,max_iter=700,step=50.0,C=9,M=4,H=2,rho=140.0,krep=.65,w_turn=.55,w_risk=.9):
    """APF-guided MultiRRT tree search with a shared exact validator.

    APF-MultiRRT and RAL-MultiRRT use the same tree expansion, APF candidate generator,
    budgets, and refinement. RAL differs only in candidate ranking: finite-horizon rollout
    scoring replaces the one-step local score. Random-node selection retains global exploration
    without using A* or grid recovery.
    """
    nodes=[start]; parents=[-1]
    goal_idx=None
    for it in range(max_iter):
        # RRT-style global exploration: goal-directed most of the time, random free-space sample otherwise.
        if rng.random() < 0.45:
            target=goal
        else:
            target=(rng.uniform(world.rs,world.w-world.rs),rng.uniform(world.rs,world.h-world.rs))
            # a blocked random sample is still a useful directional target, so no rejection is needed here.
        idx=nearest(nodes,target); p=nodes[idx]; pi=parents[idx]; prev=nodes[pi] if pi>=0 else None
        cand=candidate_set(world,p,goal,prev,rng,stats,C,step,rho,krep)
        if not cand:
            continue
        vals=[]
        for q in cand:
            if lookahead:
                v=rollout_score(world,p,q,goal,prev,rng,stats,M,H,step,rho,krep,w_turn,w_risk)
            else:
                v=local_score(world,p,q,goal,prev,rho,w_turn,w_risk)
            # A small exploration term ties local APF decisions to the current RRT target.
            v += 0.08*math.dist(q,target)/max(world.w,world.h)
            vals.append(v)
        # Add the best candidate, and occasionally one diverse runner-up to keep the tree branching.
        order=list(np.argsort(vals))
        add_indices=[order[0]]
        if len(order)>2:
            add_indices.append(order[min(2,len(order)-1)])
        for kk in add_indices:
            q=cand[int(kk)]
            # Avoid nearly duplicate tree nodes.
            if nodes and min(math.dist(q,z) for z in nodes) < step*0.22:
                continue
            nodes.append(q); parents.append(idx); qidx=len(nodes)-1; stats.expanded_nodes+=1
            chain=reconstruct(nodes,parents,qidx)
            if math.dist(q,goal) < 1e-8:
                goal_idx=qidx; break
            if math.dist(q,goal)<=step and world.segment_ok(q,goal,stats):
                if len(chain)<2 or turn_angle(chain[-2],chain[-1],goal)<=math.radians(100)+GEOM_EPS:
                    nodes.append(goal); parents.append(qidx); goal_idx=len(nodes)-1
                    break
        if goal_idx is not None:
            break
    if goal_idx is None:
        stats.failure_reason='iteration_budget'; return None
    path=reconstruct(nodes,parents,goal_idx)
    refined=safe_refine(world,path,rng,stats)
    if not path_valid(world,refined,stats):
        stats.failure_reason='validator_reject'; return None
    return refined

def plan_path(world,algorithm,start,goal,seed,params=None):
    params=params or {}; rng=random.Random(seed); stats=Stats(); t=time.perf_counter()
    if algorithm=='MultiRRT': p=rrt_plan(world,start,goal,rng,stats,star=False,smart=False,**{k:v for k,v in params.items() if k in ['max_iter','step','goal_bias']})
    elif algorithm=='RRTStar': p=rrt_plan(world,start,goal,rng,stats,star=True,smart=False,**{k:v for k,v in params.items() if k in ['max_iter','step','goal_bias']})
    elif algorithm=='RRTStarSmart': p=rrt_plan(world,start,goal,rng,stats,star=True,smart=True,**{k:v for k,v in params.items() if k in ['max_iter','step','goal_bias']})
    elif algorithm=='APFMultiRRT': p=apf_plan(world,start,goal,rng,stats,lookahead=False,**{k:v for k,v in params.items() if k in ['max_iter','step','C','M','H','rho','krep','w_turn','w_risk']})
    elif algorithm=='RALMultiRRT': p=apf_plan(world,start,goal,rng,stats,lookahead=True,**{k:v for k,v in params.items() if k in ['max_iter','step','C','M','H','rho','krep','w_turn','w_risk']})
    else: raise ValueError(algorithm)
    runtime=time.perf_counter()-t
    valid=bool(p) and path_valid(world,p,stats)
    if p and not valid: stats.failure_reason='validator_reject'
    return p,valid,runtime,stats

def resample(path,spacing=15.0):
    if not path:return []
    seg=[math.dist(a,b) for a,b in zip(path[:-1],path[1:])]; total=sum(seg)
    if total<1e-9:return [path[0]]
    targets=np.arange(0,total+1e-9,spacing); targets=np.append(targets,total) if targets[-1]<total else targets
    out=[]; cum=0; i=0
    for s in targets:
        while i<len(seg)-1 and cum+seg[i]<s: cum+=seg[i]; i+=1
        if seg[i]<1e-9: out.append(path[i]); continue
        t=(s-cum)/seg[i]; a,b=path[i],path[i+1]; out.append((a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t))
    return out

def metrics(world,path):
    if not path:return {'path_length':np.nan,'turning_angle':np.nan,'curvature_proxy':np.nan,'min_clearance':np.nan,'node_count':0}
    L=sum(math.dist(a,b) for a,b in zip(path[:-1],path[1:]))
    rr=resample(path,15.0)
    ang=[turn_angle(rr[i-1],rr[i],rr[i+1]) for i in range(1,len(rr)-1)]
    curv=[a/15.0 for a in ang]
    clear=[world.clearance(p) for p in rr]
    return {'path_length':L,'turning_angle':float(np.mean(ang)) if ang else 0.0,'curvature_proxy':float(np.mean(curv)) if curv else 0.0,'min_clearance':float(np.min(clear)) if clear else np.nan,'node_count':len(path)}
