import sys,json,math,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import app,session,net
from predictor import capacity

class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.client=app.test_client()
        r=cls.client.post('/api/command',json={'action':'reset','seed':917,'cav':.3,'scale':1.1})
        assert r.status_code==200,r.json
    @classmethod
    def tearDownClass(cls):session.close()

    def test_01_real_network(self):
        self.assertEqual(len(net.edges),149);self.assertEqual(len(net.signals),8)
        self.assertEqual(len(net.route_pool),2046)
        self.assertEqual(len(net.source['commit']),40)
    def test_02_paired_initial_state(self):
        a,b=session.worlds
        self.assertEqual(a.states,b.states)
        self.assertEqual(a.total_departed,b.total_departed)
        for w in (a,b):self.assertEqual(w.total_departed-w.total_arrived,w.conn.vehicle.getIDCount())
    def test_03_trained_prediction(self):
        s=self.client.get('/api/state').json
        for group in ('baseline','adaptive'):
            f=s['forecasts'][group];self.assertTrue(f['trained']);self.assertEqual(len(f['rows']),149)
            for e,rows in f['rows'].items():
                self.assertEqual([r['minute']for r in rows],list(range(5,61,5)))
                for r in rows:
                    for key in ('flow','speed','density','capacity','service','demand'):self.assertTrue(math.isfinite(r[key])and r[key]>=0)
                    self.assertLessEqual(r['density'],145)
    def test_04_disturbance_controls_sumo(self):
        target=net.ids[0];original=session.worlds[0].conn.lane.getMaxSpeed(net.edges[target]['lanes'][0])
        r=self.client.post('/api/command',json=dict(action='event',edge=target,type='accident',severity=.8,duration=900));self.assertEqual(r.status_code,200)
        for w in session.worlds:
            self.assertLess(w.conn.lane.getMaxSpeed(net.edges[target]['lanes'][0]),original)
            self.assertTrue(w.events[-1]['active'])
        self.client.post('/api/command',json=dict(action='clear'))
        for w in session.worlds:self.assertAlmostEqual(w.conn.lane.getMaxSpeed(net.edges[target]['lanes'][0]),original)
    def test_05_environment_and_autonomy(self):
        e=next(iter(net.edges.values()))
        self.assertGreater(capacity(e,.8),capacity(e,.1))
        self.assertLess(capacity(e,.3,.6),capacity(e,.3,1))
        self.client.post('/api/command',json=dict(action='weather',weather='rain'))
        for w in session.worlds:self.assertAlmostEqual(w.conn.vehicletype.getTau('human'),1.3/.8)
        self.client.post('/api/command',json=dict(action='weather',weather='clear'))
    def test_06_real_controller_and_conservation(self):
        self.client.post('/api/command',json=dict(action='control',enabled=True))
        with session.lock:
            for w in session.worlds:w.step(180)
            a,b=session.worlds
            self.assertFalse(a.controlled);self.assertTrue(b.controlled);self.assertGreater(b.actions,0)
            for tid in net.signals:self.assertEqual(b.conn.trafficlight.getProgram(tid),'supply_predictive')
            for w in session.worlds:
                self.assertEqual(w.teleports,0);self.assertEqual(w.collisions,0)
                self.assertEqual(w.total_departed-w.total_arrived,w.conn.vehicle.getIDCount())
            session.publish(True)
    def test_07_export_and_guardrails(self):
        r=self.client.get('/api/export.csv');self.assertEqual(r.status_code,200)
        self.assertEqual(len(r.data.decode('utf-8-sig').strip().splitlines()),1+149*12*2)
        r=self.client.post('/api/command',json=dict(action='event',edge='unknown',type='accident'));self.assertEqual(r.status_code,400)
        r=self.client.post('/api/command',json=dict(action='reset',cav=2));self.assertEqual(r.status_code,400)
        r=self.client.post('/api/command',json=dict(action='play'),headers={'Origin':'http://external.invalid'});self.assertEqual(r.status_code,403)
        r=self.client.post('/api/command',data='action=play');self.assertEqual(r.status_code,415)

if __name__=='__main__':unittest.main(verbosity=2)
