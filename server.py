import atexit, json, csv, io, threading, argparse, hashlib, os
from flask import Flask, jsonify, request, send_from_directory, Response
from config import ROOT, MODEL
from network import Network
from engine import Session

app=Flask(__name__,static_folder=str(ROOT/'web'),static_url_path='/assets')
net=Network();session=Session(net)
BUILD=hashlib.sha256((ROOT/'engine.py').read_bytes()).hexdigest()[:12]
atexit.register(session.close)

@app.get('/api/health')
def health():return jsonify(app='zhiluheng_demo',version='1.0',build=BUILD,root=str(ROOT),status=session.status)

@app.post('/api/shutdown')
def shutdown():
    if not request.is_json or request.get_json().get('app')!='zhiluheng_demo':return jsonify(error='invalid shutdown request'),400
    origin=request.headers.get('Origin')
    if origin and origin!=request.host_url.rstrip('/'):return jsonify(error='same origin required'),403
    def stop():
        session.close()
        os._exit(0)
    threading.Timer(.3,stop).start()
    return jsonify(ok=True)

@app.get('/')
def index():return send_from_directory(ROOT/'web','index.html')

@app.get('/api/network')
def network():
    p=net.payload();p['recommended']=json.loads((ROOT/'scenario/recommended_edges.json').read_text());return jsonify(p)

@app.get('/api/state')
def state():
    # Immutable published snapshots let polling stay responsive during stepping/reset.
    if not session.cached:return jsonify(status=session.status,error=session.error)
    return jsonify(session.cached)

@app.post('/api/command')
def command():
    if not request.is_json:return jsonify(error='请使用 JSON 请求'),415
    origin=request.headers.get('Origin')
    if origin and origin!=request.host_url.rstrip('/'):return jsonify(error='仅允许本地同源操作'),403
    b=request.get_json()
    try:
        if b.get('action')=='reset':
            cav=float(b.get('cav',.3));scale=float(b.get('scale',1.1));seed=int(b.get('seed',42))
            if not (0<=cav<=1 and .3<=scale<=3 and 0<=seed<=2**31-1):raise ValueError('车辆比例、需求强度或种子超出范围')
        return jsonify(session.command(b))
    except (ValueError,KeyError,TypeError)as e:return jsonify(error=str(e)),400
    except Exception as e:
        import traceback;traceback.print_exc();return jsonify(error=str(e)),500

@app.get('/api/export')
def export():
    body=json.dumps(session.cached,ensure_ascii=False,indent=2)
    return Response(body,mimetype='application/json',headers={'Content-Disposition':'attachment; filename=zhiluheng_session.json'})

@app.get('/api/export.csv')
def export_csv():
    buf=io.StringIO();wr=csv.writer(buf);wr.writerow(['group','origin_s','edge','horizon_min','flow_veh_h','speed_km_h','density_veh_km_lane','capacity_veh_h'])
    for group,f in session.forecasts.items():
        for edge,rows in f['rows'].items():
            for r in rows:wr.writerow([group,f['origin'],edge,r['minute'],r['flow'],r['speed'],r['density'],r['capacity']])
    return Response('\ufeff'+buf.getvalue(),mimetype='text/csv',headers={'Content-Disposition':'attachment; filename=forecast.csv'})

@app.get('/api/validation')
def validation():
    from config import OUT
    return jsonify(forecast=json.loads((MODEL/'metrics.json').read_text(encoding='utf-8'))if(MODEL/'metrics.json').exists()else{},
                   control=json.loads((OUT/'validation_summary.json').read_text(encoding='utf-8'))if(OUT/'validation_summary.json').exists()else[])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8878);a=p.parse_args()
    print(f'智路衡本地演示：http://127.0.0.1:{a.port}',flush=True)
    app.run(host='127.0.0.1',port=a.port,threaded=True,use_reloader=False)
