"""Shared environment-aware graph state model and learned multi-horizon residual.

Capacity priors are explicit assumptions; forecasts are conditional, not observations.
The lightweight random-feature network is fitted only on independent SUMO runs.
"""
import json, math
import numpy as np
from config import MODEL

JAM=145.0

def capacity(edge, cav, weather=1., severity=0.):
    # Mixed headway: weighted human/connected following pairs; all units veh/h.
    h=(1-cav)**2*2.1 + 2*cav*(1-cav)*1.8 + cav*cav*1.35
    lane_factor=max(.06,1-severity)
    return 3600/h*edge['nlanes']*weather*lane_factor

def features(net, states, history, eid, env, minute):
    e=net.edges[eid];s=states[eid]
    hist=list(history)[-15:]
    rows=[h[eid]for h in hist if eid in h]
    recent=rows[-5:] or [s]; older=rows[:5] or [s]
    def avg(rs,key):return sum(x.get(key,0)for x in rs)/len(rs)
    ns=[states[x]for x in e['incoming']+e['outgoing'] if x in states] or [s]
    cap=s.get('capacity',capacity(e,env['cav'],env['weather']))
    return np.array([
        s['flow']/1500,s['speed']/max(1,e['speed']*3.6),s['density']/JAM,s['queue']/30,
        avg(recent,'flow')/1500,avg(recent,'speed')/max(1,e['speed']*3.6),avg(recent,'density')/JAM,
        avg(older,'flow')/1500,avg(older,'speed')/max(1,e['speed']*3.6),avg(older,'density')/JAM,
        avg(ns,'flow')/1500,avg(ns,'density')/JAM,avg(ns,'queue')/30,
        e['length']/500,e['nlanes']/3,e['speed']/20,cap/5000,
        env['cav'],env['weather'],s.get('severity',0),float(bool(e['tls'])),
        min(4,s.get('demand',s['flow'])/max(cap,1)),minute/60,
        float(env.get('controlled',False)), min(1,len(rows)/15),
    ],dtype=np.float64)

def physical(net,states,eid,env,minute):
    e=net.edges[eid];s=states[eid]
    cap=s['capacity'];d=s.get('demand',s['flow'])
    green=s.get('green_share',1.)
    service=cap*green
    incoming_storage=sum(states[x]['queue']for x in e['incoming'])
    target_n=max(0,s['n']+(d-service)*min(minute,10)/60)
    target_n=min(JAM*e['length']/1000*e['nlanes'],target_n)
    k=target_n/max(.001,e['length']/1000*e['nlanes'])
    free=e['speed']*3.6*env['weather']*(1-.85*s.get('severity',0))
    eq_v=max(.4,free*(1-min(.985,k/JAM))**1.6)
    blend=1-math.exp(-minute/12)
    v=(1-blend)*s['speed']+blend*eq_v
    q=min(service,max(0,d+min(s['queue']+incoming_storage*.15,30)*60/max(minute,5)))
    return np.array([q,v,k])

class Predictor:
    def __init__(self,net):
        self.net=net;self.model=None;self.meta={}
        p=MODEL/'forecast.npz'
        if p.exists():
            self.model=dict(np.load(p));self.meta=json.loads((MODEL/'metrics.json').read_text(encoding='utf-8'))
    def predict(self,states,history,env,origin):
        result={}
        X=[];base=[];keys=[]
        for eid in self.net.ids:
            for h in range(5,61,5):
                X.append(features(self.net,states,history,eid,env,h));base.append(physical(self.net,states,eid,env,h));keys.append((eid,h))
        X=np.array(X);base=np.array(base)
        if self.model is not None:
            m=self.model;z=np.clip((X-m['mean'])/m['scale'],-6,6)
            phi=np.concatenate([np.ones((len(X),1)),z,np.tanh(z@m['random_w']+m['random_b'])],axis=1)
            out=base+(phi@m['beta'])*np.array([1500,60,145])
            # Physical envelopes enforce nonnegative speed/flow/density; no fabricated UI colour.
        else: out=base
        for i,(eid,h) in enumerate(keys):
            e=self.net.edges[eid];s=states[eid]
            q=float(np.clip(out[i,0],0,capacity(e,env['cav'])*1.1))
            v=float(np.clip(out[i,1],0,e['speed']*3.6*1.15))
            k=float(np.clip(out[i,2],0,JAM))
            err=self.meta.get('mae',{'flow':0,'speed':0,'density':0})
            result.setdefault(eid,[]).append(dict(minute=h,flow=round(q,1),speed=round(v,1),density=round(k,1),capacity=round(s['capacity'],1),demand=round(s.get('demand',s['flow']),1),
                flow_error=round(err['flow'],1),speed_error=round(err['speed'],1),density_error=round(err['density'],1)))
        for eid,rows in result.items():
            e=self.net.edges[eid];s=states[eid]
            for i,r in enumerate(rows):
                receiving=min([max(0,1-result[b][i]['density']/JAM)for b in e['outgoing']]or[1])
                r['service']=round(r['capacity']*s.get('green_share',1)*receiving,1)
                storage_growth=max(0,(r['density']-s['density'])*e['length']/1000*e['nlanes'])
                r['demand']=round(max(r['flow'],r['flow']+storage_growth*60/r['minute']),1)
                r['gap']=round(r['service']-r['demand'],1)
        return dict(origin=origin,rows=result,trained=self.model is not None,
                    note='未来12个5分钟窗口的统计预测；基于当前观测、天气、控制模式和训练场景的事件演化分布。未知的新事件无法预知。误差为独立仿真测试平均绝对误差，不是置信区间。',metrics=self.meta)
