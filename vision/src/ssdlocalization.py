from geometry import *
from collections import defaultdict
from geometry_msgs.msg import Pose
from math import log

MAX_OBS_PER_CLASS = 40  # to ensure database doesn't explode in size if matching fails
MIN_HEIGHT = 0.1
DEGREES90 = 1.57 #in radians

# tolerances for matches ie percentage differences
S_TOLERANCE = 0.12
H_TOLERANCE = 0.25

# keeps data for unmatched sightings, including camera transform
class Observation:
    id_count = 0

    def __init__(self, camera, class_label, x_ndc, y_ndc, extant_x, extant_y):
        self.id = Observation.id_count
        Observation.id_count += 1

        self.camera = camera
        self.class_label = class_label

        screen_x, screen_y = camera.ndc_to_screen_space(x_ndc, y_ndc)
        self.bearing = camera.get_bearing(screen_x, screen_y)
        self.screenPos = make_vector2(screen_x, screen_y)

        self.extant_x = extant_x
        self.extant_y = extant_y
        self.obj_scalar = extant_y * extant_x
        self.is_matched = False

    def print_me(self):
        print(self.camera.transform, self.id, self.bearing.transpose(),
              (self.extant_x, self.extant_y), self.screenPos.transpose())

    def position(self):
        return self.camera.position()

    def get_pose(self):
        p = Pose()
        vector_to_msg(self.position(), p.position)
        vector_to_msg(self.bearing, p.orientation)
        return p

    @classmethod
    def from_ros_msg(cls, camera, msg):
        return cls(camera, msg.class_label, average(msg.xmin, msg.xmax), 1 - average(msg.ymin, msg.ymax),
                   0.5 * (msg.xmax - msg.xmin),  0.5 * (msg.ymax - msg.ymin))

    def is_matching(self, proposal):
        proposed_pos = proposal[0]
        proposed_height = proposal[1]
        screen_vec = self.camera.project_point_to_screen_space_flat(proposed_pos) - self.screenPos
        #screen_vec = make_vector2(x, y) - self.screenPos
        screen_dist_score = np.max(np.abs(screen_vec)) < S_TOLERANCE
        v1 = proposed_pos - self.position()
        projected_height = dot(v1, self.camera.forward()) * self.extant_y * 4 * self.camera.scale_y

        height_score = abs(log(projected_height / proposed_height)) < H_TOLERANCE if projected_height > 0 else False
        return screen_dist_score and height_score and projected_height > MIN_HEIGHT

# stores Match data in the database
class Match:
    id_count = 0

    def __init__(self, estimated_pos, estimated_height):
        self.id = Match.id_count
        Match.id_count += 1
        self.estimated_pos = None
        self.estimated_height = None
        self.pose = Pose()
        self.revise_estimate(estimated_pos, estimated_height)
        self.best_angle = 0.0
        self.observations = []
        self.time_stamp = 0.0

    @classmethod
    def from_observations(cls, observation1, observation2, proposal):
        match = cls(proposal[0], proposal[1])
        observation1.is_matched = True
        observation2.is_matched = True
        match.observations = [observation1, observation2]
        match.best_angle = abs(math.acos(dot(observation1.bearing, observation2.bearing)))
        return match

    def revise_estimate(self, estimated_pos, estimated_height):
        self.estimated_pos = estimated_pos
        self.estimated_height = estimated_height
        self.pose.position.x = estimated_pos[0, 0]
        self.pose.position.y = estimated_pos[1, 0]
        self.pose.position.z = estimated_pos[2, 0]

    def add_observation(self, observation, proposal):
        observation.is_matched = True
        self.time_stamp = observation.camera.time_stamp
        if self.observations:
            obs, best_cos = self.maximum_angle_observation(observation)
            angle = abs(math.acos(best_cos))
            if angle > self.best_angle:
                self.best_angle = angle
                self.observations = [obs, observation]
                self.revise_estimate(proposal[0], proposal[1])
        else:
            self.observations.append(observation)

    def conviction(self):
        return min(1.0, self.best_angle / DEGREES90)

    def maximum_angle_observation(self, observation):
        best_obs = self.observations[0]
        best_cos = abs(dot(best_obs.bearing, observation.bearing))
        for obs in self.observations[1:]:
            dot_cos = abs(dot(obs.bearing, observation.bearing))
            if dot_cos < best_cos:
                best_cos = dot_cos
                best_obs = obs
        return best_obs, best_cos


# hold dictionaries of all localized objects and sightings
class Database:
    def __init__(self):
        self.matches = defaultdict(list)
        self.observations = defaultdict(list)

    def match_observations(self, new_observations):
        id_set = set()
        for obs in new_observations:
            self.try_to_match(obs)
            id_set.add(obs.class_label)
        # purges matched observations from dictionary and keeps list limited to max size
        for id in id_set:
            self.observations[id] = [obs for obs in self.observations[id] if not obs.is_matched]
            if len(self.observations[id]) > MAX_OBS_PER_CLASS:
                del self.observations[id][0:len(self.observations[id]) - MAX_OBS_PER_CLASS]

    def try_to_match(self, new_observation):
        for match in self.matches[new_observation.class_label]:
            if is_with_match(new_observation, match):
                return
        for old_observation in self.observations[new_observation.class_label]:
            if self.is_with_observation(new_observation, old_observation):
                return
        self.observations[new_observation.class_label].append(new_observation)

    def add_match(self, class_label, match):
        print("matched " + class_label)
        self.matches[class_label].append(match)

    def is_with_observation(self, new_obs, old_obs):
        proposal = []
        # new_obs.print_me()
        # old_obs.print_me()
        if propose_position(new_obs, old_obs, proposal) and new_obs.is_matching(proposal):
            self.add_match(new_obs.class_label, Match.from_observations(new_obs, old_obs, proposal))
            return True
        return False

    def num_of_matches(self, class_label):
        return len(self.matches[class_label])

# returns true if observation can be matched with a Match object in database
def is_with_match(new_obs, old_match):
    if old_match.conviction() > 0.5:
        proposal = [old_match.estimated_pos, old_match.estimated_height]
    else:
        other_obs, dummy = old_match.maximum_angle_observation(new_obs)
        proposal = []
        if not propose_position(new_obs, other_obs, proposal):
            return False
    if new_obs.is_matching(proposal):
        old_match.add_observation(new_obs, proposal)
        return True
    return False


# uses two observations to propose a 3D position and object height based on geometric analysis
def propose_position(obs1, obs2, proposal):
    camera2pos = obs2.position()
    origin = obs1.camera.project_point_to_screen_space(camera2pos)
    ray = obs1.camera.world_to_focal_plane[:, 0:3].dot(obs2.bearing)
    A = origin[0:2, 0:1]
    B = ray[0:2, 0:1]
    c = origin[2, 0]
    d = ray[2, 0]
    V = c * B - d * A
    denom = dot(V, obs1.screenPos * d - B)
    if abs(denom) < SMALLNUM:
        return False
    t = dot(V, A - obs1.screenPos * c) / denom
    if t < 0:  # or (c + d * t) < 0:
        return False
    proposed_position = camera2pos + obs2.bearing * t
    v2 = proposed_position - camera2pos
    projected_height = dot(v2, obs2.camera.forward()) * obs2.extant_y * 4 * obs2.camera.scale_y
    #print("projections", dot(proposed_position - obs1.position(), obs1.camera.forward()), dot(v2, obs2.camera.forward()))
    proposal.append(proposed_position)
    proposal.append(projected_height)
    return True


def vector_to_msg(vector, msg):
    msg.x = vector[0]
    msg.y = vector[1]
    msg.z = vector[2]
    return msg
