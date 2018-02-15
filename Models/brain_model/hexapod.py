# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np

n_sensors = 2
n_outputs = 6

sensors = sim.Population(n_sensors, cellclass=sim.IF_curr_exp())
outputs = sim.Population(n_outputs, cellclass=sim.IF_curr_exp())

weights = np.zeros(shape=(n_sensors, n_outputs)) # TODO

proj = sim.Projection(sensors, outputs, sim.AllToAllConnector(), sim.StaticSynapse(weight=weights))

circuit = sensors + outputs
