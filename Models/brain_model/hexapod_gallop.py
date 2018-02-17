# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np

n_sensors = 120
n_legs = 6

sensors = sim.Population(n_sensors, cellclass=sim.IF_curr_exp())
outputs_swing = [sim.Population(1, cellclass=sim.IF_curr_exp()) for i in range(n_legs)]
outputs_lift = [sim.Population(1, cellclass=sim.IF_curr_exp()) for i in range(n_legs)]

projs = []
for i in range(n_legs):
    # Swing
    if i in [0, 1]:
        weight = [2.0 for k in range(n_sensors // 6)] # cos_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_2
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_2
    elif i in [2, 3]:
        weight = [0.0 for k in range(n_sensors // 6)] # cos_phase_0
        weight.extend([2.0 for k in range(n_sensors // 6)]) # cos_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_2
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_2
    else:
        weight = [0.0 for k in range(n_sensors // 6)] # cos_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_1
        weight.extend([2.0 for k in range(n_sensors // 6)]) # cos_phase_2
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_2
    projs.append(sim.Projection(sensors, outputs_swing[i], sim.AllToAllConnector(), sim.StaticSynapse(weight=np.array(weight).reshape((n_sensors, 1)))))

    # Lift
    if i in [0, 1]:
        weight = [0.0 for k in range(n_sensors // 6)] # cos_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_2
        weight.extend([1.0 for k in range(n_sensors // 6)]) # sin_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_2
    elif i in [2, 3]:
        weight = [0.0 for k in range(n_sensors // 6)] # cos_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_2
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_0
        weight.extend([1.0 for k in range(n_sensors // 6)]) # sin_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_2
    else:
        weight = [0.0 for k in range(n_sensors // 6)] # cos_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_1
        weight.extend([0.0 for k in range(n_sensors // 6)]) # cos_phase_2
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_0
        weight.extend([0.0 for k in range(n_sensors // 6)]) # sin_phase_1
        weight.extend([1.0 for k in range(n_sensors // 6)]) # sin_phase_2
    projs.append(sim.Projection(sensors, outputs_lift[i], sim.AllToAllConnector(), sim.StaticSynapse(weight=np.array(weight).reshape((n_sensors, 1)))))

circuit = sensors
for i in range(n_legs):
    circuit += outputs_swing[i] 
    circuit += outputs_lift[i]
