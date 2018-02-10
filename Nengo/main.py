from pybrain.rl.learners.valuebased.interface import ActionValueNetwork
from pybrain.rl.learners import Q
from pybrain.rl.agents import LearningAgent
from pybrain.rl.experiments import Experiment
from RLEnvironment import RLEnvironment
from RLTask import RLTask

NUM_INTERACTIONS = 100

def main():
    environment = RLEnvironment()
    dim_state = environment.joint_states.shape[0]
    num_actions = len(environment.actions)
    controller = ActionValueNetwork(dim_state, num_actions)
    learner = Q()
    agent = LearningAgent(controller, learner)
    task = RLTask(environment)
    experiment = Experiment(task, agent)
    
    while True:
        experiment.doInteractions(NUM_INTERACTIONS)
        agent.learn()
        agent.reset()

if __name__ == "__main__":
    main()