@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.n_sensors), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapVariable("n_sensors", initial_value=nrp.config.brain_root.n_sensors)
@nrp.Robot2Neuron()
def robot_to_neuron(t, sensors, n_sensors):
    amp = 5
    for i in range(n_sensors.value // 2):
        sensors[i].amplitude = amp * 1.0
    import math
    for i in range(n_sensors.value // 2, n_sensors.value):
        sensors[i].amplitude = amp * (math.sin(2*t) + 1) / 2.0
