from __future__ import print_function
from pybrain.rl.learners.valuebased.interface import ActionValueNetwork
from pybrain.rl.learners.valuebased import SARSA
from pybrain.rl.agents import LearningAgent
from pybrain.rl.experiments import Experiment
from RLEnvironment import RLEnvironment
from RLTask import RLTask
import rospy

NUM_INTERACTIONS = 10

def main():
    rospy.init_node("lauron_reinforcement_learning")
    environment = RLEnvironment()
    dim_state = environment.joint_states.shape[0]
    num_actions = len(environment.actions)
    controller = ActionValueNetwork(dim_state, num_actions)
    learner = SARSA()
    agent = LearningAgent(controller, learner)
    task = RLTask(environment)
    experiment = Experiment(task, agent)
    
    episode_counter = 0
    while True:
        print("Training episode {}".format(episode_counter))
        experiment.doInteractions(NUM_INTERACTIONS)
        agent.learn()
        agent.reset()
        episode_counter += 1

if __name__ == "__main__":
    main()