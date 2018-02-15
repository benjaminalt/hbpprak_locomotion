from pybrain.rl.environments.task import Task
from rospy import Subscriber
from gazebo_msgs.msg import ModelStates
from collections import defaultdict, deque
import numpy as np
import time

class RLTask(Task):
    episode_time = 60 # seconds

    def __init__(self, environment):
        self.robot_position_sub = Subscriber("/gazebo/model_states", ModelStates, self.on_position_update)
        self.current_pos = np.zeros(shape=3)
        self.last_poses = deque(3 * [np.zeros(3)], maxlen=5)
        self.start_time = time.time()
        super(RLTask, self).__init__(environment)

    def on_position_update(self, msg):
        self.current_pos = np.array([msg.pose[2].position.x, msg.pose[2].position.y, msg.pose[2].position.z])

    #def isFinished(self):
    #    return time.time() - self.start_time > self.episode_time
    
    def getReward(self):
        self.last_poses.append(np.copy(self.current_pos))
        return np.array([(np.linalg.norm(self.last_poses[-1] - self.last_poses[0]))])
    
    #def getObservation(self):
    #    obs = super(RLTask, self).getObservation()
    #    return obs
    