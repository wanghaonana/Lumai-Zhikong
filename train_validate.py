"""Reproducible simulation training, held-out forecast tests and paired control tests."""
import argparse, json, time, csv, math, copy
from collections import deque
import numpy as np
from config import ROOT, OUT, MODEL
from network import Network
from engine import World
from predictor import features, physical, Predictor

def generate(net,seed,scale,cav,weather,event,controlled,duration=9000):
    path=OUT/f'train_seed{seed}.json'
    if path.exists():return json.loads(path.read_text())
    route,count=net.demand(seed=seed,scale=scale,cav=cav,seconds=duration,name=f'train_{seed}')
    w=World(net,route,seed,cav,f'train{seed}',False)
    w.set_weather(weather)
    if controlled:w.enable()
    frames=[];envs=[]
    targets=json.loads((ROOT/'scenario/recommended_edges.json').read_text())
    target=targets[seed%5]['id']
    try:
        for t in range(0,duration,60):
            if event and t in (1200,4200):w.incident(target,'accident'if seed%2 else'works',1200,.75+.05*(seed%3))
            w.step(60)
            frames.append(copy.deepcopy(w.states));envs.append(w.environment())
        result=dict(seed=seed,frames=frames,envs=envs,metrics=w.snapshot()['metrics'],route_count=count,settings=dict(scale=scale,cav=cav,weather=weather,event=event,controlled=controlled))
        path.write_text(json.dumps(result,separators=(',',':')))
        print('generated',seed,'frames',len(frames),'metrics',result['metrics'],flush=True)
        return result
    finally:w.close()

def samples(net,runs,max_per_run=16000):
    X=[];Y=[];B=[];persist=[]
    for run in runs:
        rng=np.random.default_rng(run['seed']);frames=run['frames'];envs=run['envs']
        candidates=[]
        for t in range(14,len(frames)-60,3):
            hist=frames[max(0,t-14):t+1]
            for eid in net.ids:
                for h in (5,10,15,20,25,30,35,40,45,50,55,60):candidates.append((t,eid,h))
        if len(candidates)>max_per_run:candidates=[candidates[i]for i in rng.choice(len(candidates),max_per_run,replace=False)]
        for t,eid,h in candidates:
            s=frames[t];hist=frames[max(0,t-14):t+1];env=envs[t]
            # Future target window only supplies labels, never input features.
            target=frames[t+h-4:t+h+1]
            y=np.mean([[f[eid]['flow'],f[eid]['speed'],f[eid]['density']]for f in target],axis=0)
            X.append(features(net,s,hist,eid,env,h));Y.append(y);B.append(physical(net,s,eid,env,h))
            persist.append([s[eid]['flow'],s[eid]['speed'],s[eid]['density']])
    return np.array(X),np.array(Y),np.array(B),np.array(persist)

def train(net):
    # Entire simulation runs, including RNG and events, are separated before fitting.
    settings=[(11,.8,.1,'clear',False,False),(12,1.1,.3,'rain',True,False),
              (13,1.4,.6,'clear',True,True),(14,1.6,.1,'storm',False,True),
              (15,1.25,.8,'rain',True,True),(16,1.,.5,'clear',True,False)]
    runs=[generate(net,*s)for s in settings]
    validation=[generate(net,51,1.2,.4,'rain',True,False),generate(net,52,1.45,.5,'clear',True,True)]
    test=[generate(net,81,1.3,.35,'clear',True,False),generate(net,82,1.2,.65,'rain',True,True)]
    X,Y,B,P=samples(net,runs);XV,YV,BV,PV=samples(net,validation,7000);XT,YT,BT,PT=samples(net,test,9000)
    mean=X.mean(0);scale=X.std(0);scale[scale<.02]=.02
    rng=np.random.default_rng(177);rw=rng.normal(0,.22,(X.shape[1],64));rb=rng.normal(0,.5,64)
    def design(x):
        z=np.clip((x-mean)/scale,-6,6);return np.c_[np.ones(len(z)),z,np.tanh(z@rw+rb)]
    A=design(X);AV=design(XV);AT=design(XT)
    norm=np.array([1500,60,145]);target=(Y-B)/norm
    best=None
    for reg in (1,10,100,1000):
        beta=np.linalg.solve(A.T@A+np.eye(A.shape[1])*reg,A.T@target)
        pred=BV+AV@beta*norm
        error=np.mean(abs(pred-YV)/norm)
        if best is None or error<best[0]:best=(error,reg,beta)
    _,reg,beta=best
    np.savez_compressed(MODEL/'forecast.npz',mean=mean,scale=scale,random_w=rw,random_b=rb,beta=beta)
    pred=BT+AT@beta*norm
    pcav=XT[:,17];pcap=3600/((1-pcav)**2*2.1+2*pcav*(1-pcav)*1.8+pcav**2*1.35)*XT[:,14]*3*1.1
    pred[:,0]=np.maximum(0,np.minimum(pred[:,0],pcap));pred[:,1]=np.maximum(0,np.minimum(pred[:,1],XT[:,15]*20*3.6*1.15));pred[:,2]=np.clip(pred[:,2],0,145)
    names=['flow','speed','density']
    def metric(a,b):return dict(zip(names,[round(float(x),3)for x in np.mean(abs(a-b),0)]))
    metrics=dict(model='Graph-state random-feature residual + physical supply model',training_seeds=[x[0]for x in settings],validation_seeds=[51,52],test_seeds=[81,82],
        samples_train=len(X),samples_test=len(XT),horizon_minutes=list(range(5,61,5)),ridge=reg,
        mae=metric(pred,YT),persistence_mae=metric(PT,YT),physical_mae=metric(BT,YT),
        note='仅为独立随机种子的合成交通仿真误差；不是现场精度。训练、验证、测试整次仿真隔离。',
        per_horizon={str(h):metric(pred[XT[:,22]==h/60],YT[XT[:,22]==h/60])for h in range(5,61,5)})
    active=(YT[:,0]>0)|(YT[:,2]>0)
    metrics['active_samples']=int(active.sum())
    metrics['active_mae']=metric(pred[active],YT[active])
    metrics['active_persistence_mae']=metric(PT[active],YT[active])
    metrics['active_physical_mae']=metric(BT[active],YT[active])
    (MODEL/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding='utf-8')
    print('FORECAST_TEST',json.dumps(metrics,ensure_ascii=False),flush=True)
    return metrics

def evaluate(net,seed=101,kind='accident',scale=1.25,cav=.3,weather='clear',duration=2400,target=None):
    route,_=net.demand(seed=seed,scale=scale,cav=cav,seconds=duration,name=f'eval_{seed}_{kind}')
    target=target or json.loads((ROOT/'scenario/recommended_edges.json').read_text())[0]['id']
    worlds=[World(net,route,seed,cav,n,False)for n in ('eval_base','eval_ctrl')]
    timeline=[];predictor=Predictor(net);initial_equal=None
    try:
        for w in worlds:w.set_weather(weather)
        for t in range(0,duration,10):
            if t==600:
                initial_equal=worlds[0].states==worlds[1].states
                for w in worlds:w.incident(target,kind,1500,.88)
            if t==900:worlds[1].enable()
            for w in worlds:w.step(10)
            if t%60==0:
                for w in worlds:
                    if w.controlled:
                        fc=predictor.predict(w.states,w.history,w.environment(),w.now)
                        w.short_demand={e:r[0]['flow']for e,r in fc['rows'].items()}
            if t%30==0:
                timeline.append(dict(t=t+10,baseline=worlds[0].snapshot()['metrics'],adaptive=worlds[1].snapshot()['metrics'],
                    target_base=worlds[0].states[target],target_ctrl=worlds[1].states[target]))
        a,b=[w.snapshot()['metrics']for w in worlds]
        result=dict(seed=seed,type=kind,target=target,scale=scale,cav=cav,weather=weather,duration=duration,initial_equal=initial_equal,baseline=a,adaptive=b,
                    delay_reduction_pct=round(100*(a['delay']-b['delay'])/max(1,a['delay']),2),
                    queue_reduction_pct=round(100*(a['queue_integral']-b['queue_integral'])/max(1,a['queue_integral']),2),timeline=timeline)
        (OUT/f'evaluation_{seed}_{kind}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print('CONTROL_TEST',json.dumps({k:v for k,v in result.items()if k!='timeline'},ensure_ascii=False),flush=True)
        return result
    finally:
        for w in worlds:w.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['train','test','quick'],default='train');a=p.parse_args();n=Network()
    if a.mode=='train':train(n)
    elif a.mode=='quick':evaluate(n,duration=1800)
    else:
        cases=[(201,'accident','clear',.3),(211,'accident','clear',.3),(221,'accident','clear',.3),
               (202,'works','rain',.6),(212,'works','rain',.6),(222,'works','rain',.6),
               (203,'surge','clear',.1),(213,'surge','clear',.1),(223,'surge','clear',.1)]
        results=[evaluate(n,seed=seed,kind=kind,weather=weather,cav=cav)for seed,kind,weather,cav in cases]
        (OUT/'validation_summary.json').write_text(json.dumps([{k:v for k,v in r.items()if k!='timeline'}for r in results],ensure_ascii=False,indent=2),encoding='utf-8')
