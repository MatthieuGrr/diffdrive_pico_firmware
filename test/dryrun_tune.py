import sys, types, time
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from faux_robot import FauxPico, Roue
import serial
G=Roue(3.2,0.18,70); D=Roue(2.9,0.22,95)
faux=FauxPico(G,D)
serial.Serial=lambda *a,**k: faux
import tune_pid
print(f"ROBOT SIMULE : G K=3.20 tau=180ms zm=70 | D K=2.90 tau=220ms zm=95",flush=True)
print("(le script doit retrouver ces valeurs puis converger)\n",flush=True)
try: tune_pid.main()
except SystemExit as e: print(f"[sortie code {e.code}]")
