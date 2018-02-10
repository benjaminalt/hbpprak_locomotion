from pybrain.rl.environments.environment import Environment
from rospy import Subscriber
from sensor_msgs.msg import JointState
from collections import defaultdict
from motionPrimitives import MotionPrimitives
import numpy as np
import nengo

class RLEnvironment(Environment):
    joint_names = ["robot_leg0_alpha_joint", "robot_leg0_beta_joint", "robot_leg0_delta_joint", "robot_leg0_gamma_joint",
            "robot_leg1_alpha_joint", "robot_leg1_beta_joint", "robot_leg1_delta_joint", "robot_leg1_gamma_joint",
            "robot_leg2_alpha_joint", "robot_leg2_beta_joint", "robot_leg2_delta_joint", "robot_leg2_gamma_joint",
            "robot_leg3_alpha_joint", "robot_leg3_beta_joint", "robot_leg3_delta_joint", "robot_leg3_gamma_joint",
            "robot_leg4_alpha_joint", "robot_leg4_beta_joint", "robot_leg4_delta_joint", "robot_leg4_gamma_joint",
            "robot_leg5_alpha_joint", "robot_leg5_beta_joint", "robot_leg5_delta_joint", "robot_leg5_gamma_joint"]
    num_legs = 6
    num_primitives = 3
    action_run_time = 1 # seconds

    def __init__(self):
        """
        Set up the actions and sensors of this environment.
        The neural network simulation must be started by the client!
        """
        self.joint_state_subscriber = Subscriber("/robot/joints", JointState, self.on_joint_states_changed)
        self.joint_states = np.zeros(len(self.joint_names), dtype=np.float)
        self.actions = []
        self.primitives = []
        self.stimuli = np.zeros(shape=(self.num_legs, self.num_primitives))
        
        # Set up actions
        for leg_idx in range(self.num_legs):
            for primitive_idx in range(self.num_primitives):
                # Add action for each possible stimulus
                for value in range(10):
                    self.actions.append(self.make_move_func(leg_idx, primitive_idx, value * 0.1))
                # Add primitive
                if primitive_idx == 0:
                    joint_topics = [["/robot_leg{}_alpha_joint_pos_cntr/command".format(leg_idx)], ["/robot_leg{}_delta_joint_pos_cntr/command".format(leg_idx)]]
                elif primitive_idx == 1:
                    joint_topics = [["/robot_leg{}_beta_joint_pos_cntr/command".format(leg_idx)], ["/robot_leg{}_gamma_joint_pos_cntr/command".format(leg_idx)]]
                elif primitive_idx == 2:
                    joint_topics = [["/robot_leg{}_beta_joint_pos_cntr/command".format(leg_idx)]]
                self.primitives.append(MotionPrimitives(primitive_idx, lambda : self.stimuli[leg_idx, primitive_idx], joint_topics))
        self.primitives = np.reshape(np.array(self.primitives), (self.num_legs,self.num_primitives))

        self.model = nengo.Network()
        with self.model:
            for leg_idx in range(self.num_legs):
                for primitive_idx in range(self.num_primitives):
                    self.model.add(self.primitives[leg_idx, primitive_idx].get_network())

    def make_move_func(self, leg_idx, primitive_idx, stimulus):
        return lambda : self.move(leg_idx, primitive_idx, stimulus)

    def move(self, leg_idx, primitive_idx, stimulus):
        self.stimuli[leg_idx, primitive_idx] = stimulus

    def on_joint_states_changed(self, msg):
        self.joint_states = np.array(msg.data.position[:-2])

    def performAction(self, action):
        """
        Set a stimulus for the chosen action (leg & motion primitive).
        Runs a short nengo simulation to execute the primitives.
        """
        self.actions[action]()

        # Start nengo simulation to move the joints
        with nengo.Simulator(self.model) as sim:
            sim.run(self.action_run_time)
    
    def getSensors(self):
        return self.joint_states
    
    def reset(self):
        pass