import os, sys, time, math, json, random, socket, subprocess, threading, copy
from collections import deque, defaultdict
from pathlib import Path
from config import ROOT, SUMO, NET, OUT
import traci
import traci.constants as tc
from predictor import capacity, JAM, Predictor

class World:
    def __init__(self,net,route,seed=42,cav=.3,name='baseline',render=True,edge_output=None):
        self.net=net;self.cav=cav;self.name=name;self.render=render
        self.weather=1.;self.controlled=False;self.now=0;self.events=[];self.logs=deque(maxlen=50)
        self.entered=defaultdict(deque);self.prev={e:set()for e in net.ids}
        self.history=deque(maxlen=120);self.states={};self.vehicles=[]
        self.total_delay=0.;self.pending_delay=0.;self.total_departed=0;self.total_arrived=0;self.travel_sum=0.;self.start_times={}
        self.teleports=0;self.collisions=0;self.phase_seen={};self.last_reroute=-60;self.rerouted=0;self.actions=0;self.controllers={}
        self.phase_green={};self.short_demand={};self.queue_integral=0.;self.peak_queue=0
        self.green_observations=defaultdict(lambda:deque(maxlen=180))
        self.run_id=f'{name}_{time.time_ns()}'
        self.logfile=open(OUT/f'{self.run_id}.log','w',encoding='utf-8')
        with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
        args=[SUMO,'-n',str(NET),'-r',str(route),'--begin','0','--end','20000','--step-length','1','--seed',str(seed),
              '--time-to-teleport','-1','--no-step-log','true','--duration-log.disable','true','--no-warnings','true',
              '--remote-port',str(port),'--collision.action','warn','--save-state.rng','true']
        if edge_output:
            add=OUT/f'{self.run_id}.add.xml'
            add.write_text(f'<additional><edgeData id="edges" period="60" file="{Path(edge_output).as_posix()}" excludeEmpty="false"/></additional>')
            args+=['-a',str(add)]
        self.proc=subprocess.Popen(args,stdout=self.logfile,stderr=self.logfile,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        self.conn=traci.connect(port,numRetries=15,proc=self.proc)
        for eid in net.ids:
            self.conn.edge.subscribe(eid,[tc.LAST_STEP_VEHICLE_NUMBER,tc.LAST_STEP_MEAN_SPEED,tc.LAST_STEP_VEHICLE_HALTING_NUMBER,tc.LAST_STEP_VEHICLE_ID_LIST])
        for lane in {l['lane']for t in net.signals.values()for l in t['links']}:
            self.conn.lane.subscribe(lane,[tc.LAST_STEP_VEHICLE_NUMBER,tc.LAST_STEP_VEHICLE_HALTING_NUMBER])
        for tid in net.signals:self.conn.trafficlight.subscribe(tid,[tc.TL_RED_YELLOW_GREEN_STATE])
        self._collect()

    def environment(self):return dict(cav=self.cav,weather=self.weather,controlled=self.controlled)

    def severity(self,eid):
        return max([ev['severity']for ev in self.events if ev['active'] and ev['edge']==eid and ev['type'] in ('accident','works')]or[0])

    def green_share(self,eid):
        observed=self.green_observations[eid]
        if len(observed)>=120:return max(.03,sum(observed)/len(observed))
        tls=self.net.edges[eid]['tls']
        if not tls:return 1.
        t=self.net.signals[tls];links=[l for l in t['links']if l['a']==eid]
        phases=t['phases'];cycle=sum(p['duration']for p in phases)
        return max(.08,sum(p['duration']for p in phases if any(p['state'][l['index']]in 'Gg'for l in links))/max(cycle,1))

    def _collect(self):
        values=self.conn.edge.getAllSubscriptionResults();states={}
        for eid,e in self.net.edges.items():
            r=values.get(eid,{})
            ids=set(r.get(tc.LAST_STEP_VEHICLE_ID_LIST,()))
            new=ids-self.prev[eid]
            if new:self.entered[eid].append((self.now,len(new)))
            while self.entered[eid]and self.entered[eid][0][0]<=self.now-60:self.entered[eid].popleft()
            n=int(r.get(tc.LAST_STEP_VEHICLE_NUMBER,0));q=int(r.get(tc.LAST_STEP_VEHICLE_HALTING_NUMBER,0))
            speed=max(0,r.get(tc.LAST_STEP_MEAN_SPEED,e['speed']))*3.6 if n else None
            flow=sum(c for _,c in self.entered[eid])*3600/max(1,min(self.now,60)) if self.now else 0
            density=n/max(.001,e['length']/1000*e['nlanes']);sev=self.severity(eid)
            cap=capacity(e,self.cav,self.weather,sev)
            states[eid]=dict(n=n,queue=q,speed=round(speed if speed is not None else e['speed']*3.6*self.weather,2),empty=n==0,
                flow=round(flow,2),density=round(density,2),capacity=round(cap,2),severity=sev,green_share=self.green_share(eid),demand=flow)
            self.prev[eid]=ids
        for eid,e in self.net.edges.items():
            s=states[eid]
            upstream=sum(states[x]['queue']/max(1,len(self.net.edges[x]['outgoing']))for x in e['incoming'])
            s['demand']=round(s['flow']+upstream*60/5,2)
            s['gap']=round(s['capacity']*s['green_share']-s['demand'],1)
            ratio=s['speed']/max(1,e['speed']*3.6)
            s['congestion']=round(0 if s['empty']else min(1,max(0,1-ratio)*.75+min(1,s['density']/70)*.25),3)
        self.states=states

    def set_weather(self,kind):
        self.weather={'clear':1.,'rain':.8,'storm':.6}[kind]
        for eid,e in self.net.edges.items():
            for lane in e['lanes']:self.conn.lane.setMaxSpeed(lane,e['speed']*self.weather*(1-.94*self.severity(eid)))
        # Weather changes behaviour as well as road speed, for both vehicle classes.
        for typ,tau in [('human',1.3),('connected',.85)]:self.conn.vehicletype.setTau(typ,tau/self.weather)
        self.logs.appendleft(dict(t=self.now,text=f'环境更新：{kind}；同步改变限速与跟驰时距。'))

    def incident(self,eid,kind='accident',duration=900,severity=.82):
        if eid not in self.net.edges:raise ValueError('路段不存在')
        if kind not in ('accident','works','surge'):raise ValueError('未知事件类型')
        ev=dict(id=f'ev{len(self.events)}',edge=eid,type=kind,start=self.now,end=self.now+duration,severity=severity,active=True)
        self.events.append(ev)
        if kind!='surge':
            e=self.net.edges[eid]
            for lane in e['lanes']:self.conn.lane.setMaxSpeed(lane,max(.5,e['speed']*self.weather*(1-.94*severity)))
        self.logs.appendleft(dict(t=self.now,text=f'{eid}：注入{kind}，持续{duration}秒；两组事件相同。'))
        return ev

    def remove_events(self):
        for ev in self.events:ev['active']=False
        for eid,e in self.net.edges.items():
            for lane in e['lanes']:self.conn.lane.setMaxSpeed(lane,e['speed']*self.weather)

    def enable(self,value=True):
        self.controlled=value;self.phase_seen={};self.controllers={}
        if not value:
            for tid in self.net.signals:self.conn.trafficlight.setProgram(tid,'0')
        self.logs.appendleft(dict(t=self.now,text='启用供需预测控制：优化绿灯时长并向可绕行车辆提供分流。'if value else'恢复原始固定信号方案。'))

    def control(self):
        return self._actuated_control()

    def _actuated_control(self):
        if not self.controlled:return
        for tid,t in self.net.signals.items():
            if tid not in self.controllers:
                # Let SUMO's established detector logic handle safe phase transitions.
                # Preserve source phase order and yellow durations, expose green bounds.
                current=self.conn.trafficlight.getPhase(tid)
                ps=[]
                for p in t['phases']:
                    green='y'not in p['state'].lower() and 'G'in p['state']
                    d=p['duration']
                    ps.append(traci.trafficlight.Phase(d,p['state'],minDur=max(4,d*.35)if green else d,maxDur=max(10,min(80,d*1.7))if green else d))
                logic=traci.trafficlight.Logic('supply_predictive',tc.TRAFFICLIGHT_TYPE_ACTUATED,current,ps,subParameter={'max-gap':'3.0','detector-gap':'2.0','show-detectors':'false'})
                self.conn.trafficlight.setProgramLogic(tid,logic)
                self.conn.trafficlight.setProgram(tid,'supply_predictive')
                self.controllers[tid]={'gap':3.0}
            if self.now%30:continue
            incoming=list({l['a']for l in t['links']})
            # A finite-action model predictive layer chooses detector-gap sensitivity.
            # Arrivals come from current counts + the 5-min learned forecast; receiving
            # factors prevent long extension into nearly full downstream links.
            demand=sum(max(self.states[a]['flow'],min(self.states[a]['demand'],self.short_demand.get(a,0)))for a in incoming)/3600
            q=sum(self.states[a]['queue']for a in incoming)
            cap=sum(self.states[a]['capacity']for a in incoming)/3600
            recv=min([max(.1,1-self.states[l['b']]['density']/JAM)for l in t['links']]or[1])
            scores=[]
            for gap in (1.5,2.,2.5,3.,3.5,4.):
                effective=cap*recv*(1-math.exp(-gap/2))*.45
                terminal=max(0,q+30*(demand-effective))
                # Extension lowers lost time, but long gaps delay conflicting movements.
                cost=terminal**2+gap*gap*(3+q*.5)
                scores.append((cost,gap))
            gap=min(scores)[1]
            self.conn.trafficlight.setParameter(tid,'max-gap',str(gap))
            self.controllers[tid]['gap']=gap;self.actions+=1
            if self.actions%16==0:self.logs.appendleft(dict(t=self.now,text=f'路口 {tid}：30s滚动预测选取 {gap:.1f}s 检测间隙；在原安全相位和绿灯上下界内自适应通行。'))
        if self.now-self.last_reroute>=90:
            self.last_reroute=self.now
            for eid,e in self.net.edges.items():
                s=self.states[eid];free=e['length']/max(1,e['speed']*self.weather)
                self.conn.edge.adaptTraveltime(eid,free*(1+4*s['severity'])+min(45,s['queue']))
            for vid in self.conn.vehicle.getIDList():
                if sum(vid.encode())%10>=7 or self.conn.vehicle.getRoadID(vid).startswith(':'):continue
                try:
                    old=self.conn.vehicle.getRoute(vid);remaining=old[self.conn.vehicle.getRouteIndex(vid):]
                    if not any(self.severity(e)>.1 for e in remaining if e in self.net.edges):continue
                    self.conn.vehicle.rerouteTraveltime(vid,currentTravelTimes=False)
                    if self.conn.vehicle.getRoute(vid)!=old:self.rerouted+=1
                except traci.TraCIException:pass

    def step(self,seconds=1):
        for _ in range(seconds):
            for ev in self.events:
                if ev['active']and self.now>=ev['end']:
                    ev['active']=False;e=self.net.edges[ev['edge']]
                    for lane in e['lanes']:self.conn.lane.setMaxSpeed(lane,e['speed']*self.weather*(1-.94*self.severity(ev['edge'])))
                    self.logs.appendleft(dict(t=self.now,text=f'{ev["edge"]} 事件按相同时间表结束。'))
                if ev['active']and ev['type']=='surge'and self.now%3==0:
                    routes=[r for r in self.net.route_pool if ev['edge']in r]
                    if routes:
                        route=routes[(self.now//3)%len(routes)];rid=f'{ev["id"]}_{self.now}'
                        self.conn.route.add(rid,route)
                        self.conn.vehicle.add('extra_'+rid,rid,typeID='human',departLane='best',departSpeed='max')
            self.control()
            self.conn.simulationStep();self.now=int(self.conn.simulation.getTime())
            for tid,r in self.conn.trafficlight.getAllSubscriptionResults().items():
                signal=r.get(tc.TL_RED_YELLOW_GREEN_STATE,'')
                groups=defaultdict(list)
                for l in self.net.signals[tid]['links']:groups[l['a']].append(l['index'])
                for a,indices in groups.items():self.green_observations[a].append(float(any(signal[i]in'Gg'for i in indices)))
            departed=self.conn.simulation.getDepartedIDList();arrived=self.conn.simulation.getArrivedIDList()
            self.total_departed+=len(departed);self.total_arrived+=len(arrived)
            for vid in departed:
                self.start_times[vid]=self.now
                if self.render:self.conn.vehicle.subscribe(vid,[tc.VAR_POSITION,tc.VAR_ANGLE,tc.VAR_SPEED,tc.VAR_ROAD_ID,tc.VAR_TYPE])
            for vid in arrived:self.travel_sum+=self.now-self.start_times.pop(vid,self.now)
            self.teleports+=self.conn.simulation.getStartingTeleportNumber()
            self.collisions+=self.conn.simulation.getCollidingVehiclesNumber()
            self._collect()
            delay=sum(s['n']*max(0,1-s['speed']/max(1,self.net.edges[e]['speed']*3.6))for e,s in self.states.items())
            self.total_delay+=delay
            self.pending_delay+=len(self.conn.simulation.getPendingVehicles())
            q=sum(s['queue']for s in self.states.values());self.queue_integral+=q;self.peak_queue=max(self.peak_queue,q)
            if self.now%60==0:self.history.append(copy.deepcopy(self.states))
        if self.render:
            results=self.conn.vehicle.getAllSubscriptionResults()
            self.vehicles=[dict(id=v,x=r[tc.VAR_POSITION][0],y=r[tc.VAR_POSITION][1],angle=r[tc.VAR_ANGLE],speed=r[tc.VAR_SPEED],edge=r[tc.VAR_ROAD_ID],cav=r.get(tc.VAR_TYPE)=='connected')for v,r in results.items()if tc.VAR_POSITION in r]

    def snapshot(self):
        n=sum(s['n']for s in self.states.values());q=sum(s['queue']for s in self.states.values())
        all_vehicles=self.conn.vehicle.getIDCount()
        pending=self.conn.simulation.getPendingVehicles()
        return dict(time=self.now,states=self.states,vehicles=self.vehicles,events=self.events,logs=list(self.logs),controlled=self.controlled,
            signals={t:self.conn.trafficlight.getRedYellowGreenState(t)for t in self.net.signals},
            metrics=dict(vehicles=all_vehicles,edge_vehicles=n,queue=q,mean_speed=round(sum(s['n']*s['speed']for s in self.states.values())/max(n,1),2),
                arrived=self.total_arrived,departed=self.total_departed,pending=len(pending),delay=round(self.total_delay+self.pending_delay,1),road_delay=round(self.total_delay,1),pending_delay=round(self.pending_delay,1),
                mean_travel=round(self.travel_sum/max(1,self.total_arrived),1),teleports=self.teleports,collisions=self.collisions,
                queue_integral=round(self.queue_integral),peak_queue=self.peak_queue,actions=self.actions,rerouted=self.rerouted))

    def close(self):
        try:self.conn.close()
        except Exception:pass
        try:self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:self.proc.terminate()
        self.logfile.close()

class Session:
    def __init__(self,net):
        self.net=net;self.lock=threading.RLock();self.worlds=[];self.running=False;self.speed=12;self.error=None
        self.forecasts={};self.timeline=[];self.status='idle';self.settings=dict(seed=42,cav=.3,scale=1.1)
        self.predictor=Predictor(net);self.last_forecast=-999;self.cached={};self.stop_flag=False
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def reset(self,settings=None):
        with self.lock:
            self.running=False;self.status='loading'
            for w in self.worlds:w.close()
            self.worlds=[];self.error=None;self.forecasts={};self.timeline=[]
            self.predictor=Predictor(self.net)
            if settings:self.settings.update(settings)
            route,_=self.net.demand(**self.settings,seconds=14400,name='interactive')
            for name in ('baseline','adaptive'):self.worlds.append(World(self.net,route,self.settings['seed'],self.settings['cav'],name))
            # Same warmup, no controller, no disturbance. User then branches explicitly.
            for _ in range(900):
                for w in self.worlds:w.step()
            self.last_forecast=-999;self.status='ready';self.publish(True)
        return self.cached

    def publish(self,predict=False):
        if not self.worlds:return
        t=self.worlds[0].now
        if predict or t-self.last_forecast>=60:
            for w in self.worlds:
                p=self.predictor.predict(w.states,w.history,w.environment(),t)
                self.forecasts[w.name]=p
                w.short_demand={e:r[0]['flow']for e,r in p['rows'].items()}
            self.last_forecast=t
        snaps={w.name:w.snapshot()for w in self.worlds}
        if not self.timeline or t-self.timeline[-1]['t']>=10:
            self.timeline.append(dict(t=t,baseline=snaps['baseline']['metrics'],adaptive=snaps['adaptive']['metrics']))
        self.cached=dict(status=self.status,running=self.running,speed=self.speed,settings=self.settings,time=t,worlds=snaps,forecasts=self.forecasts,timeline=self.timeline[-720:],error=self.error)

    def command(self,b):
        action=b.get('action')
        if action=='reset':return self.reset(dict(seed=int(b.get('seed',42)),cav=float(b.get('cav',.3)),scale=float(b.get('scale',1.1))))
        with self.lock:
            if not self.worlds:raise ValueError('请先初始化仿真')
            if action=='play':self.running=True;self.status='running'
            elif action=='pause':self.running=False;self.status='paused'
            elif action=='speed':self.speed=max(1,min(80,int(b['speed'])))
            elif action=='control':self.worlds[1].enable(bool(b.get('enabled',True)))
            elif action=='event':
                duration=max(60,min(7200,int(b.get('duration',900))))
                severity=max(.1,min(.98,float(b.get('severity',.82))))
                for w in self.worlds:w.incident(b['edge'],b.get('type','accident'),duration,severity)
            elif action=='weather':
                for w in self.worlds:w.set_weather(b.get('weather','clear'))
            elif action=='clear':
                for w in self.worlds:w.remove_events()
            elif action=='forecast':pass
            else:raise ValueError('未知操作')
            self.publish(action in ('forecast','control','event','weather','clear'))
            return {'ok':True,'time':self.worlds[0].now}

    def loop(self):
        while not self.stop_flag:
            started=time.monotonic()
            try:
                with self.lock:
                    if self.running and self.worlds:
                        for _ in range(max(1,self.speed//4)):
                            for w in self.worlds:w.step()
                        if self.worlds[0].now>=14400:self.running=False;self.status='complete'
                        self.publish()
            except Exception as exc:
                self.error=str(exc);self.running=False;self.status='error'
                self.cached={**self.cached,'error':self.error,'running':False,'status':'error'}
                import traceback;traceback.print_exc()
            time.sleep(max(.01,.25-(time.monotonic()-started)))

    def close(self):
        self.stop_flag=True
        with self.lock:
            for w in self.worlds:w.close()
