from pathlib import Path
import os, sys
ROOT = Path(__file__).resolve().parent
SUMO_HOME = ROOT / 'runtime' / 'sumo'
os.environ['SUMO_HOME'] = str(SUMO_HOME)
sys.path.insert(0, str(SUMO_HOME / 'tools'))
SUMO = str(SUMO_HOME / 'bin' / 'sumo.exe')
ROUTER = str(SUMO_HOME / 'bin' / 'duarouter.exe')
NET = ROOT / 'scenario' / 'cologne8.net.xml'
OUT = ROOT / 'outputs'
OUT.mkdir(exist_ok=True)
MODEL = ROOT / 'models'
MODEL.mkdir(exist_ok=True)
