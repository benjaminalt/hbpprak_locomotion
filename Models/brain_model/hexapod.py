# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np

n_sensors = 120
n_legs = 6

sensors = sim.Population(n_sensors, cellclass=sim.IF_curr_exp())
outputs = [sim.Population(1, cellclass=sim.IF_curr_exp()) for i in range(n_legs)]

projs = []
for i in range(n_legs):
    if i in [0, 3, 4]:
        weight = [1.0 for k in range(n_sensors // 2)]
        weight.extend([-1.0 for k in range(n_sensors // 2)])
    else:
        weight = [0.0 for k in range(n_sensors // 2)]
        weight.extend([1.0 for k in range(n_sensors // 2)])
    projs.append(sim.Projection(sensors, outputs[i], sim.AllToAllConnector(), sim.StaticSynapse(weight=np.array(weight).reshape((n_sensors, 1)))))

circuit = sensors
for i in range(n_legs):
    circuit += outputs[i]
