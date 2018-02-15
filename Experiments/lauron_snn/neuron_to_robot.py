@nrp.MapSpikeSink("output_population", nrp.brain.outputs, nrp.leaky_integrator_alpha)
@nrp.Neuron2Robot()
def neuron_to_robot(t, output_population):
    clientLogger.info(output_population.voltage)
