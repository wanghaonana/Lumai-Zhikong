import json, random, hashlib, subprocess
import xml.etree.ElementTree as ET
from collections import Counter
from config import ROOT, NET, ROUTER, OUT

class Network:
    def __init__(self):
        root = ET.parse(NET).getroot()
        self.edges = {}
        self.lane_edge = {}
        for e in root.findall('edge'):
            if e.get('function') == 'internal': continue
            lanes = [l for l in e.findall('lane') if 'pedestrian' != l.get('allow')]
            if not lanes: continue
            shapes = [[[float(v) for v in xy.split(',')] for xy in l.get('shape').split()] for l in lanes]
            rec = dict(id=e.get('id'), name=e.get('name') or e.get('id'), start=e.get('from'), end=e.get('to'),
                       lanes=[l.get('id') for l in lanes], lane_shapes=shapes, shape=shapes[0],
                       length=float(lanes[0].get('length')), speed=float(lanes[0].get('speed')),
                       nlanes=len(lanes), outgoing=[], incoming=[], tls=None)
            self.edges[rec['id']] = rec
            for l in rec['lanes']: self.lane_edge[l] = rec['id']
        self.signals = {}
        for t in root.findall('tlLogic'):
            self.signals[t.get('id')] = dict(id=t.get('id'), phases=[dict(state=p.get('state'),duration=float(p.get('duration'))) for p in t.findall('phase')], links=[])
        for c in root.findall('connection'):
            a,b=c.get('from'),c.get('to')
            if a not in self.edges or b not in self.edges: continue
            if b not in self.edges[a]['outgoing']: self.edges[a]['outgoing'].append(b)
            if a not in self.edges[b]['incoming']: self.edges[b]['incoming'].append(a)
            tls=c.get('tl')
            if tls in self.signals:
                self.edges[a]['tls']=tls
                self.signals[tls]['links'].append(dict(index=int(c.get('linkIndex')),a=a,b=b,lane=f'{a}_{c.get("fromLane")}'))
        self.ids=list(self.edges)
        self.index={e:i for i,e in enumerate(self.ids)}
        self.junctions=[dict(id=j.get('id'),x=float(j.get('x')),y=float(j.get('y')),shape=[[float(v)for v in xy.split(',')]for xy in j.get('shape','').split()],type=j.get('type')) for j in root.findall('junction') if j.get('type')!='internal']
        self.bounds=[float(x)for x in root.find('location').get('convBoundary').split(',')]
        self.source=json.loads((ROOT/'scenario/source.json').read_text(encoding='utf-8-sig'))
        self.source['sha256']=hashlib.sha256(NET.read_bytes()).hexdigest()
        self.route_pool=[]
        pool=ROOT/'scenario/route_pool.json'
        if pool.exists(): self.route_pool=json.loads(pool.read_text())

    def payload(self):
        return dict(edges=list(self.edges.values()),junctions=self.junctions,bounds=self.bounds,signals=self.signals,source=self.source,name='德国科隆 · Cologne8',algorithm='AG-SD MPC · 环境感知图供需预测控制')

    def prepare_routes(self):
        target=OUT/'routed_original.rou.xml'
        cmd=[ROUTER,'-n',str(NET),'-r',str(ROOT/'scenario/cologne8.rou.xml'),'-o',str(target),'--no-step-log','true','--ignore-errors','false','--no-warnings','true']
        subprocess.run(cmd,check=True,capture_output=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        root=ET.parse(target).getroot()
        self.route_pool=[v.find('route').get('edges').split() for v in root.findall('vehicle') if v.find('route') is not None]
        (ROOT/'scenario/route_pool.json').write_text(json.dumps(self.route_pool),encoding='utf-8')
        counts=Counter(e for r in self.route_pool for e in r)
        candidates=[e for e in self.edges if self.edges[e]['tls'] and self.edges[e]['length']>70]
        ranked=sorted(candidates,key=lambda e:counts[e],reverse=True)
        (ROOT/'scenario/recommended_edges.json').write_text(json.dumps([dict(id=e,count=counts[e],lanes=self.edges[e]['nlanes'],length=self.edges[e]['length'])for e in ranked[:12]],indent=2))
        return len(self.route_pool), ranked[:8]

    def demand(self, seed=42, scale=1.0, cav=.3, seconds=10800, name='demand'):
        rng=random.Random(seed)
        root=ET.Element('routes')
        ET.SubElement(root,'vType',id='human',vClass='passenger',carFollowModel='IDM',tau='1.3',minGap='2.0',length='4.5',accel='2.3',decel='4.5',speedDev='.08')
        ET.SubElement(root,'vType',id='connected',vClass='passenger',carFollowModel='IDM',tau='.85',minGap='1.5',length='4.5',accel='2.5',decel='4.5',speedDev='.03')
        # Origin-destination samples come from the published scenario; departures are synthetic.
        t=0.; i=0
        while t<seconds:
            rate=(2046/3600)*scale*(1+.18*__import__('math').sin(t/1800*__import__('math').pi))
            t+=rng.expovariate(rate)
            if t>=seconds: break
            route=rng.choice(self.route_pool)
            v=ET.SubElement(root,'vehicle',id=f'v{i}',type='connected' if rng.random()<cav else 'human',depart=f'{t:.2f}',departLane='best',departSpeed='max')
            ET.SubElement(v,'route',edges=' '.join(route)); i+=1
        path=OUT/f'{name}.rou.xml'
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        return path,i

if __name__=='__main__':
    n=Network();print(n.prepare_routes());print(n.demand())
