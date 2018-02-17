@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.n_sensors), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapVariable("n_sensors", initial_value=nrp.config.brain_root.n_sensors)
@nrp.Robot2Neuron()
def robot_to_neuron_gallop(t, sensors, n_sensors):
    import math
    amp = 5
    freq = 10
    phase_shift = 2.0 * math.pi / 3.0 
    for i in range(n_sensors.value // 6):
        sensors[i].amplitude = amp * (math.cos(freq * t) + 1) / 2.0
    for i in range(n_sensors.value // 6, 2 * n_sensors.value // 6):
        sensors[i].amplitude = amp * (math.cos(freq * t + phase_shift) + 1) / 2.0
    for i in range(2 * n_sensors.value // 6, 3 * n_sensors.value // 6):
        sensors[i].amplitude = amp * (math.cos(freq * t + 2 * phase_shift) + 1) / 2.0
    for i in range(3 * n_sensors.value // 6, 4 * n_sensors.value // 6):
        sensors[i].amplitude = amp * (math.sin(freq * t) + 1) / 2.0
    for i in range(4 * n_sensors.value // 6, 5 * n_sensors.value // 6):
        sensors[i].amplitude = amp * (math.sin(freq * t + phase_shift) + 1) / 2.0
    for i in range(5 * n_sensors.value // 6, n_sensors.value):
        sensors[i].amplitude = amp * (math.sin(freq * t + 2 * phase_shift) + 1) / 2.0