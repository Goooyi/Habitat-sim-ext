# Import necessary modules, define some convenience functions, and initialize the Simulator and Agent.
import argparse
import json
import os
import math
import random
import time
from pathlib import Path

import magnum as mn
import numpy as np
from tkinter import W
from PIL import Image
from PIL.ImageFilter import GaussianBlur

import habitat_sim
import habitat_sim.physics
from habitat_sim.utils import common as ut
from habitat_sim.utils import viz_utils as vut
from pedestrian import PedestrianPathFollower
   
def quaternion_to_list(q: np.quaternion):
    return q.imag.tolist() + [q.real]

# object seen thres
thres = 0.20

dir_path = Path(__file__).absolute().parents[0]
print(dir_path)

# TODO: data_path to Path class
data_path = os.path.join(dir_path, "data")
# load config.json that is in the same folder
conf = os.path.join(dir_path, "config.json")
with open(conf) as file:
    config = json.load(file)


# define some globals the first time we run.
if "sim" not in globals():
    global sim
    sim = None
    global obj_attr_mgr
    obj_attr_mgr = None
    global prim_attr_mgr
    prim_attr_mgr = None
    global stage_attr_mgr
    stage_attr_mgr = None
    global rigid_obj_mgr
    rigid_obj_mgr = None

# Define Configuration Utility Functions
def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene_id = settings["scene"]
    sim_cfg.enable_physics = settings["enable_physics"]

    # Note: all sensors must have the same resolution
    sensor_specs = []
    if settings["color_sensor_1st_person"]:
        color_sensor_1st_person_spec = habitat_sim.CameraSensorSpec()
        color_sensor_1st_person_spec.uuid = "rgba_camera_1stperson"
        color_sensor_1st_person_spec.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor_1st_person_spec.resolution = [
        settings["height"],
            settings["width"],
        ]
        color_sensor_1st_person_spec.position = [0.0,settings["sensor_height"], 0.0]
        color_sensor_1st_person_spec.orientation = [
            settings["sensor_pitch"],
            0.0,
            0.0,
        ]
        color_sensor_1st_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(color_sensor_1st_person_spec)
    if settings["depth_sensor_1st_person"]:
        depth_sensor_1st_person_spec = habitat_sim.CameraSensorSpec()
        depth_sensor_1st_person_spec.uuid = "depth_camera_1stperson"
        depth_sensor_1st_person_spec.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor_1st_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        depth_sensor_1st_person_spec.position = [0.0, settings["sensor_height"], 0.0]
        depth_sensor_1st_person_spec.orientation = [
            settings["sensor_pitch"],
            0.0,
            0.0,
        ]
        depth_sensor_1st_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(depth_sensor_1st_person_spec)
    if settings["semantic_sensor_1st_person"]:
        semantic_sensor_1st_person_spec = habitat_sim.CameraSensorSpec()
        semantic_sensor_1st_person_spec.uuid = "semantic_sensor_1stperson"
        semantic_sensor_1st_person_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
        semantic_sensor_1st_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        semantic_sensor_1st_person_spec.position = [
            0.0,
            settings["sensor_height"],
            0.0,
        ]
        semantic_sensor_1st_person_spec.orientation = [
            settings["sensor_pitch"],
            0.0,
            0.0,
        ]
        semantic_sensor_1st_person_spec.sensor_subtype = (
            habitat_sim.SensorSubType.PINHOLE
        )
        sensor_specs.append(semantic_sensor_1st_person_spec)
    if settings["color_sensor_3rd_person"]:
        color_sensor_3rd_person_spec = habitat_sim.CameraSensorSpec()
        color_sensor_3rd_person_spec.uuid = "rgba_camera_3rdperson"
        color_sensor_3rd_person_spec.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor_3rd_person_spec.resolution = [
            settings["height"],
            settings["width"],
        ]
        color_sensor_3rd_person_spec.position = [
            0.0,
            settings["sensor_height"] + 0.2,
            0.2,
        ]
        color_sensor_3rd_person_spec.orientation = [-math.pi / 4, 0, 0]
        color_sensor_3rd_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(color_sensor_3rd_person_spec)

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs
    return habitat_sim.Configuration(sim_cfg, [agent_cfg])

# TODO: refactor, should pass config?
def make_default_settings():
    settings = {
        "width": config["width"],#  480, Spatial resolution of the observations
        "height": config["height"], # 270,
        "scene": os.path.join(dir_path,config["scene"]), # Scene path 
        "default_agent": 0,
        "sensor_height": config["sensor_height"],  # Height of sensors in meters
        "sensor_pitch": -math.pi / 8.0,  # sensor pitch (x rotation in rads)
        "color_sensor_1st_person": config["color_sensor_1st_person"],  # RGB sensor
        "color_sensor_3rd_person": config["color_sensor_3rd_person"],  # RGB sensor 3rd person
        "depth_sensor_1st_person": config["depth_sensor_1st_person"],  # Depth sensor
        "semantic_sensor_1st_person": config["semantic_sensor_1st_person"],  # Semantic sensor
        "seed": config["seed"],
        "enable_physics": config["enable_physics"],  # enable dynamics simulation
    }
    return settings

def make_simulator_from_settings(sim_settings):
    cfg = make_cfg(sim_settings)
    # clean-up the current simulator instance if it exists
    global sim
    global obj_attr_mgr
    global prim_attr_mgr
    global stage_attr_mgr
    global rigid_obj_mgr
    if sim != None:
        sim.close()
    # initialize the simulator
    sim = habitat_sim.Simulator(cfg)
    # Managers of various Attributes templates
    obj_attr_mgr = sim.get_object_template_manager()
    obj_attr_mgr.load_configs(str(os.path.join(data_path, "objects/locobot_merged")))
    prim_attr_mgr = sim.get_asset_template_manager()
    stage_attr_mgr = sim.get_stage_template_manager()
    # Manager providing access to rigid objects
    rigid_obj_mgr = sim.get_rigid_object_manager()

# Define Simulation Utility Functions
def random_change_brightness(img, value=5):
    intensity = random.randint(-value,value)
    source = img.split()
    R, G, B = 0, 1, 2
    Red = source[R].point(lambda i: i+intensity)

    Green = source[G].point(lambda i: i+intensity)

    Blue = source[B].point(lambda i: i+intensity)

    image = Image.merge("RGB", (Red, Green, Blue))

    return image

def add_gaussian_noise(image, mean=0, std=5):
    """
    Args:
        image : numpy array of image
        mean : pixel mean of image
        standard deviation : pixel standard deviation of image
    Return :
        image : numpy array of image with gaussian noise added
    """
    gaus_noise = np.random.normal(mean, std, (480,640))
    gaus_noise = gaus_noise[:,:,None]
    # image = image.astype("int16")
    noise_img = image + gaus_noise
    image = ceil_floor_image(image)
    return noise_img 

def ceil_floor_image(image):
    """
    Args:
        image : numpy array of image in datatype int16
    Return :
        image : numpy array of image in datatype uint8 with ceilling(maximum 255) and flooring(minimum 0)
    """
    image[image > 255] = 255
    image[image < 0] = 0
    # image = image.astype("uint8")
    return image


def gausian_blur(img, radius=3):
    image = img.filter(GaussianBlur(radius))
    return image

def remove_all_objects(sim):
    for obj_id in sim.get_existing_object_ids():
        sim.remove_object(obj_id)

def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    sstart_time = sim.get_world_time()
    while sim.get_world_time() < sstart_time + dt:
        sim.step_physics(1.0 / float(config["frame_rate"]))
        if get_frames:
            observations.append(sim.get_sensor_observations())
    return observations

# sample a random valid state for the object from the scene bounding box or navmesh
def sample_object_state(
    sim, object_id, from_navmesh=True, maintain_object_up=True, max_tries=100, bb=None
):
    # check that the object is not STATIC
    if sim.get_object_motion_type(object_id) is habitat_sim.physics.MotionType.STATIC:
        print("sample_object_state : Object is STATIC, aborting.")
    if from_navmesh:
        if not sim.pathfinder.is_loaded:
            print("sample_object_state : No pathfinder, aborting.")
            return False
    elif not bb:
        print(
            "sample_object_state : from_navmesh not specified and no bounding box provided, aborting."
        )
        return False
    tries = 0
    valid_placement = False
    # Note: following assumes sim was not reconfigured without close
    scene_collision_margin = obj_attr_mgr.get_template_by_id(0).margin
    while not valid_placement and tries < max_tries:
        tries += 1
        # initialize sample location to random point in scene bounding box
        sample_location = np.array([0, 0, 0])
        if from_navmesh:
            # query random navigable point
            sample_location = sim.pathfinder.get_random_navigable_point()
        else:
            sample_location = np.random.uniform(bb.min, bb.max)
        # set the test state
        sim.set_translation(sample_location, object_id)
        if maintain_object_up:
            # random rotation only on the Y axis
            y_rotation = mn.Quaternion.rotation(
                mn.Rad(random.random() * 2 * math.pi), mn.Vector3(0, 1.0, 0)
            )
            sim.set_rotation(y_rotation * sim.get_rotation(object_id), object_id)
        else:
            # unconstrained random rotation
            sim.set_rotation(ut.random_quaternion(), object_id)

        # raise object such that lowest bounding box corner is above the navmesh sample point.
        if from_navmesh:
            obj_node = sim.get_object_scene_node(object_id)
            xform_bb = habitat_sim.geo.get_transformed_bb(
                obj_node.cumulative_bb, obj_node.transformation
            )
            # also account for collision margin of the scene
            y_translation = mn.Vector3(
                0, xform_bb.size_y() / 2.0 + scene_collision_margin, 0
            )
            sim.set_translation(
                y_translation + sim.get_translation(object_id), object_id
            )

        # test for penetration with the environment
        if not sim.contact_test(object_id):
            valid_placement = True

    if not valid_placement:
        return False,0
    # destination = sim.get_translation(object_id)
    return True

if __name__ == "__main__":
    #TODO
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.add_argument("--video-prefix", dest="video_prefix", default=None, help="setting the data category and video prefix")
    parser.add_argument("--video-time", dest="video_time", default=None, help="setting the video time", type=int)
    parser.add_argument("--seed", dest="seed", default=None, type=int)
    parser.add_argument("--speed", dest="speed", default=None, type=int)
    parser.add_argument("--scene", dest="scene", default=None)
    parser.add_argument("--pedestrians", nargs="+",  dest="pedestrians", default=None)
    parser.add_argument("--non-dynamic", dest="nonDynamic", action="store_true")
    parser.set_defaults(show_video=False, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = False
    display = args.display
    make_video = args.make_video
    if args.nonDynamic:
        config["nonDynamic"] = True
    if args.video_prefix:
        config["video_prefix"] = args.video_prefix
    if args.video_time:
        config["video_time"] = args.video_time
    if args.seed:
        config["seed"] = args.seed
    if args.scene:
        config["scene"] = args.scene
    if args.pedestrians:
        config["pedestrians"] = args.pedestrians
    if args.speed:
        config["pedestrian_max_turn_speed"] = [args.speed]* len(config["pedestrians"])
        config["pedestrian_max_linear_speed"] = [args.speed]* len(config["pedestrians"])
    # object name to semantic id mapping
    semantic_ids = [config["semantic_mapping_name2id"][i] for i in config["pedestrians"]]
else:
    show_video = False
    make_video = False
    display = False

# Continuous Path Follower for agent 
class ContinuousPathFollower(object):
    def __init__(self, sim, path, agent_scene_node, waypoint_threshold):
        self._sim = sim
        self._points = path.points[:]
        assert len(self._points) > 0
        self._length = path.geodesic_distance
        self._node = agent_scene_node
        self._threshold = waypoint_threshold
        self._step_size = 0.01
        self.progress = 0  # geodesic distance -> [0,1]
        self.waypoint = path.points[0]

        # setup progress waypoints
        _point_progress = [0]
        _segment_tangents = []
        _length = self._length
        for ix, point in enumerate(self._points):
            if ix > 0:
                segment = point - self._points[ix - 1]
                segment_length = np.linalg.norm(segment)
                segment_tangent = segment / segment_length
                _point_progress.append(
                    segment_length / _length + _point_progress[ix - 1]
                )
                # t-1 -> t
                _segment_tangents.append(segment_tangent)
        self._point_progress = _point_progress
        self._segment_tangents = _segment_tangents
        # final tangent is duplicated
        self._segment_tangents.append(self._segment_tangents[-1])

        # print("self._length = " + str(self._length))
        # print("num points = " + str(len(self._points)))
        # print("self._point_progress = " + str(self._point_progress))
        # print("self._segment_tangents = " + str(self._segment_tangents))

    def pos_at(self, progress):
        if progress <= 0:
            return self._points[0]
        elif progress >= 1.0:
            return self._points[-1]

        path_ix = 0
        for ix, prog in enumerate(self._point_progress):
            if prog > progress:
                path_ix = ix
                break

        segment_distance = self._length * (progress - self._point_progress[path_ix - 1])
        return (
            self._points[path_ix - 1]
            + self._segment_tangents[path_ix - 1] * segment_distance
        )

    def update_waypoint(self):
        if self.progress < 1.0:
            wp_disp = self.waypoint - self._node.translation
            wp_dist = np.linalg.norm(wp_disp)
            node_pos = self._node.translation
            step_size = self._step_size
            threshold = self._threshold
            while wp_dist < threshold:
                self.progress += step_size
                self.waypoint = self.pos_at(self.progress)
                if self.progress >= 1.0:
                    break
                wp_disp = self.waypoint - node_pos
                wp_dist = np.linalg.norm(wp_disp)

def track_waypoint(waypoint, rs, vc, dt=1.0 / 60.0):
    angular_error_threshold = 0.5
    max_linear_speed = config["agent_max_linear_speed"]
    max_turn_speed = config["agent_max_turn_speed"]
    glob_forward = rs.rotation.transform_vector(mn.Vector3(0, 0, -1.0)).normalized()
    glob_right = rs.rotation.transform_vector(mn.Vector3(-1.0, 0, 0)).normalized()
    to_waypoint = mn.Vector3(waypoint) - rs.translation
    u_to_waypoint = to_waypoint.normalized()
    angle_error = float(mn.math.angle(glob_forward, u_to_waypoint))

    new_velocity = 0
    # print("angle_error！！！！！: %2.6f" % angle_error)

    if angle_error < angular_error_threshold:
        # speed up to max
        new_velocity = (vc.linear_velocity[2] - max_linear_speed) / 2.0
    else:
        # slow down to 0
        new_velocity = (vc.linear_velocity[2]) / 2.0
        # new_velocity = 0
    vc.linear_velocity = mn.Vector3(0, 0, new_velocity)

    # angular part
    rot_dir = 1.0
    
    if mn.math.dot(u_to_waypoint,glob_right) < 0:
    # if mn.math.cross(u_to_waypoint,glob_forward)[1] >0  :
        rot_dir = -1.0
    # print(rot_dir) ##
    # print(mn.math.angle(glob_forward, u_to_waypoint))
    # print(mn.math.cross(u_to_waypoint,glob_forward))
    angular_correction = 0.0
    if angle_error > (max_turn_speed * 10.0 * dt):
        angular_correction = max_turn_speed
    else:
        angular_correction = angle_error / 2.0

    vc.angular_velocity = mn.Vector3(
        0, np.clip(rot_dir * angular_correction, -max_turn_speed, max_turn_speed), 0
    )

def setup_path_visualization(sim, path_follower, vis_samples=100):
    vis_ids = []
    sphere_handle = obj_attr_mgr.get_template_handles("uvSphereSolid")[0]
    sphere_template_cpy = obj_attr_mgr.get_template_by_handle(sphere_handle)
    sphere_template_cpy.scale *= 0.2
    template_id = obj_attr_mgr.register_template(sphere_template_cpy, "mini-sphere")
    print("template_id = " + str(template_id))
    if template_id < 0:
        return None
    vis_ids.append(sim.add_object_by_handle(sphere_handle))

    for point in path_follower._points:
        cp_id = sim.add_object_by_handle(sphere_handle)
        if cp_id < 0:
            print(cp_id)
            return None
        sim.set_translation(point, cp_id)
        vis_ids.append(cp_id)

    for i in range(vis_samples):
        cp_id = sim.add_object_by_handle("mini-sphere")
        if cp_id < 0:
            print(cp_id)
            return None
        sim.set_translation(path_follower.pos_at(float(i / vis_samples)), cp_id)
        vis_ids.append(cp_id)

    for obj_id in vis_ids:
        if obj_id < 0:
            print(obj_id)
            return None

    for obj_id in vis_ids:
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, obj_id)

    return vis_ids

# Embodied Continuous Navigation to find a sampled object
# TODO: refactor, sim_settings will passed to output handle, to save camera psecs
sim_settings = make_default_settings()
# fmt: off

make_simulator_from_settings(sim_settings)

# navmesh settings
default_nav_mesh_settings = habitat_sim.NavMeshSettings()
default_nav_mesh_settings.set_defaults()
inflated_nav_mesh_settings = habitat_sim.NavMeshSettings()
inflated_nav_mesh_settings.set_defaults()
# TODO: give user the option to set this if collision happened, so before this, must give collisin warning
inflated_nav_mesh_settings.agent_radius = 0.35
inflated_nav_mesh_settings.agent_height = 1.4
# min:0.01, max:0.5, step:0.01, default = 0.2
inflated_nav_mesh_settings.agent_max_climb = 0.02
# min:0, max:85, step:1.0,default = 45.0
inflated_nav_mesh_settings.agent_max_slope = 20.0
# sim.config.sim_cfg.allow_sliding = True
recompute_successful = sim.recompute_navmesh(sim.pathfinder, inflated_nav_mesh_settings)
if not recompute_successful:
    print("Failed to recompute navmesh!")
    raise RuntimeError('Failed to recompute navmesh!')
navmesh_settings = habitat_sim.NavMeshSettings()

# Load the selected object, here we use locobot and place it on the NavMesh
locobot_template_id = obj_attr_mgr.load_configs(
    str(os.path.join(data_path, "objects/locobot_merged")),False
)[0]

locobot_template_handle = obj_attr_mgr.get_file_template_handles("locobot")[0]
# add robot object to the scene with the agent/camera SceneNode attached
locobot_id = sim.add_object_by_handle(
    locobot_template_handle, sim.agents[0].scene_node
    )

# set the agent's body to kinematic since we will be updating position manually
# locobot_obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, locobot_id)

# create and configure a new VelocityControl structure
# Note: this is NOT the object's VelocityControl, so it will not be consumed automatically in sim.step_physics
vel_control = habitat_sim.physics.VelocityControl()
vel_control.controlling_lin_vel = True
vel_control.lin_vel_is_local = True
vel_control.controlling_ang_vel = True
vel_control.ang_vel_is_local = True


# set fps
show_waypoint_indicators = False  # @param {type:"boolean"}
time_step = 1.0 / float(config["frame_rate"])

sim.config.sim_cfg.allow_sliding = True

video_time = config["video_time"]

# check if there is an object seen in this video:
seen = False
observations = []
# manully add 4 path and loop through this 4 path
camera_info = {"resolution": [config["width"], config["height"]], "position":[],"orientation":[]}
# the index of "positions" and "locations" keys is the semantic_id-1 of objects
peds_infos = {"positions":[list() for _ in range(len(config["pedestrians"]))], 
                "orientations":[list() for _ in range(len(config["pedestrians"]))]}
while not seen:
    # Different objects
    loaded_peds = []
    for i in range(len(config["pedestrians"])):
        loaded_peds.append(obj_attr_mgr.load_configs(str(os.path.join(data_path,"objects/"+config["pedestrians"][i])),False)[0])
    if config["nonDynamic"]:
        loaded_nonDynmaic_objs = []
        obj_model_list = [x for x in config["semantic_mapping_name2id"].keys()]
        total_model_number = len(obj_model_list)
        # already_chosed = []
        test = []
        for x in range(config["nonDynamicCount"]):
            chosen_object = obj_model_list[x % total_model_number]
            model_path = os.path.join(data_path, "objects/"+chosen_object)
            loaded_nonDynamic = obj_attr_mgr.load_configs(str(model_path),False)[0]
            added_nonDynamic = rigid_obj_mgr.add_object_by_template_id(loaded_nonDynamic)
            test.append(added_nonDynamic)
            added_nonDynamic.semantic_id = 0
            sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, added_nonDynamic.object_id)
            # random.seed(x)
            # sim.pathfinder.seed(random.randint(1,100000))
            # tmp = sim.pathfinder.get_random_navigable_point(max_tries=1000)
            # while tmp in already_chosed:
                # tmp = sim.pathfinder.get_random_navigable_point(max_tries=1000) 
            tmp = sim.pathfinder.get_random_navigable_point(max_tries=1000) 
            # already_chosed.append(tmp)
            # sim.set_translation(tmp, added_nonDynamic.object_id)
            added_nonDynamic.translation = mn.Vector3(tmp)
            # print(already_chosed)
        for i in test:
            # i.translation = mn.Vector3([1,1,1])
            print(i.translation)

    sim_settings["seed"] += 1
    seed = sim_settings["seed"]
    random.seed(seed)
    sim.seed(random.randint(1,10000000))
    # reset observations and robot state
    # locobot_obj.translation = sim.pathfinder.get_random_navigable_point()
    # init_loc = sim.pathfinder.get_random_navigable_point()

    # sim.set_translation(init_loc, locobot_id)
    sim.set_translation(sim.pathfinder.get_random_navigable_point(), locobot_id)
    peds=[]
    obj_ids = []
    for i in range(len(config["pedestrians"])):
        temp = PedestrianPathFollower(sim,config["semantic_id"][i],0.4,config["total_path_number"][i],config["pedestrian_max_linear_speed"][i],config["pedestrian_max_turn_speed"][i],rigid_obj_mgr=rigid_obj_mgr)
        temp.initial_pedestrian_path(loaded_peds[i])
        peds.append(temp)
    observations.clear()
    
    # get shortest path to the object from the agent position
    found_path = False
    path1 = habitat_sim.ShortestPath()
    path2 = habitat_sim.ShortestPath()
    path3 = habitat_sim.ShortestPath()
    path4 = habitat_sim.ShortestPath()
    path5 = habitat_sim.ShortestPath()
    while not found_path:
        origin = sim.get_translation(locobot_id)
        path1.requested_start = origin
        path1.requested_end = mn.Vector3(sim.pathfinder.get_random_navigable_point())
        path2.requested_start = mn.Vector3(path1.requested_end)
        path2.requested_end = mn.Vector3(sim.pathfinder.get_random_navigable_point())
        path3.requested_start = mn.Vector3(path2.requested_end)
        path3.requested_end = mn.Vector3(sim.pathfinder.get_random_navigable_point())
        path4.requested_start = mn.Vector3(path3.requested_end)
        path4.requested_end = mn.Vector3(sim.pathfinder.get_random_navigable_point())
        path5.requested_start = mn.Vector3(path4.requested_end)
        path5.requested_end = origin

        found_path = sim.pathfinder.find_path(path1) and sim.pathfinder.find_path(path2) and sim.pathfinder.find_path(path3) and sim.pathfinder.find_path(path4) and sim.pathfinder.find_path(path5)

    if not found_path:
        print("Could not find path to object, aborting!")
        raise RuntimeError("Could not find path to object, aborting!")

    vis_objs = []

    continuous_path_follower = ContinuousPathFollower(
        sim, path1, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.4
    )

    start_time = sim.get_world_time()
    i = 0
    while sim.get_world_time() - start_time < video_time:
        if i % 5 == 1:
            print("Path 01")
            continuous_path_follower = ContinuousPathFollower(
                sim, path2, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
            )
        elif i % 5 == 2:
            print("Path 02")
            continuous_path_follower = ContinuousPathFollower(
                sim, path3, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
            )
        elif i % 5 == 3:
            print("Path 03")
            continuous_path_follower = ContinuousPathFollower(
                sim, path4, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
            )
        elif i % 5 == 4:
            print("Path 04")
            continuous_path_follower = ContinuousPathFollower(
                sim, path5, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
            )
        else:
            print("Path 00")

        if show_waypoint_indicators:
            for vis_obj in vis_objs:
                sim.remove_object(vis_obj)
            vis_objs = setup_path_visualization(sim,continuous_path_follower)

        # manually control the object's kinematic state via velocity integration
        previous_pos = sim.get_translation(locobot_id)
        while (
            continuous_path_follower.progress < 1.0
            and sim.get_world_time() - start_time < video_time
        ):
            continuous_path_follower.update_waypoint()
            if show_waypoint_indicators:
                sim.set_translation(continuous_path_follower.waypoint, vis_objs[0])

            if locobot_id < 0:
                print("locobot_id " + str(locobot_id.object_id))
                break

            previous_rigid_state = sim.get_rigid_state(locobot_id)

            # sotre camera info :
            camera_info["position"].append(sim.get_agent(0).get_state().sensor_states["rgba_camera_1stperson"].position.tolist())
            camera_info["orientation"].append(quaternion_to_list(sim.get_agent(0).get_state().sensor_states["rgba_camera_1stperson"].rotation))
            # time.sleep(10000)
            # print(sim.get_agent(0).get_state().sensor_states["rgba_camera_1stperson"].hfov)
            # set velocities based on relative waypoint position/direction
            track_waypoint(
                continuous_path_follower.waypoint,
                previous_rigid_state,
                vel_control,
                dt=time_step,
            )

            # manually integrate the rigid state
            target_rigid_state = vel_control.integrate_transform(
                time_step, previous_rigid_state
            )

            # snap rigid state to navmesh and set state to object/agent

            end_pos = sim.step_filter(
                previous_rigid_state.translation, target_rigid_state.translation
            )

            sim.set_translation(end_pos, locobot_id)
            # sim.set_translation(target_rigid_state.translation, locobot_id)
            # print("rotation here!!!rotation here!!!rotation here!!!")
            # print(target_rigid_state.rotation)
            sim.set_rotation(target_rigid_state.rotation, locobot_id)

            # update pedestrain state and store peds infos
            for index, j in enumerate(peds):
                cur_loc = j.obj.translation 
                cur_orientation = j.obj.rotation 
                peds_infos["positions"][j.semantic_id - 1].append(np.array(cur_loc)-camera_info["position"][-1])
                peds_infos["orientations"][j.semantic_id - 1].append((np.array(cur_orientation.vector).tolist()+[cur_orientation.scalar]))
                j.integrate_state(time_step)

            # Check if a collision occured
            dist_moved_before_filter = (
                target_rigid_state.translation - previous_rigid_state.translation
            ).dot()
            dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

            # print(continuous_path_follower.progress)

            # if continuous_path_follower.progress < 1.0:
            #     sim.set_translation(end_pos, locobot_id)

            # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
            # collision _didn't_ happen. One such case is going up stairs.  Instead,
            # we check to see if the the amount moved after the application of the filter
            # is _less_ than the amount moved before the application of the filter
            EPS = 1e-5
            collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter
            # collided =  (previous_pos-sim.get_translation(locobot_id)).dot() < EPS
            # if(collided):
                # print("Warning: collided here!!")
            #     # TODO: reset or change oriantation so it can continue to run?

            previous_pos = sim.get_translation(locobot_id)

            # run any dynamics simulation
            sim.step_physics(time_step)

            # render observation
            observations.append(sim.get_sensor_observations())

            # check if seen object
            if not seen and np.sum(np.where(observations[-1]["semantic_sensor_1stperson"] != 0)) >= thres*640*480:
                seen = True
        i += 1
    for p in peds:
        sim.remove_object(p.obj_id, delete_object_node=True)


# save all results
video_prefix = config["video_prefix"]
# TODO: replace 480p with config_value
output_path = dir_path / config["output_path"]

physics_info_folder = output_path / "physics_info"
img_folder = output_path / "habitat_sim_DAVIS"

# mapping video name to index
rgb_folder = img_folder / 'JPEGImages/480p'
rgb_folder.mkdir(parents=True, exist_ok=True)
folderIndex = len([x for x in rgb_folder.iterdir() if x.is_dir()])
folderIndex = '%04i' % (folderIndex)
name_mapping = [{folderIndex : video_prefix}]

rgb_save_folder = img_folder / 'JPEGImages/480p' / folderIndex
depth_save_folder = img_folder / 'Annotations/480p_depth' / folderIndex
colored_mask_save_folder = img_folder / 'Annotations/480p_colored' / folderIndex
objectid_mask_save_folder = img_folder / 'Annotations/480p_objectID' / folderIndex
imageSets_folder = img_folder / 'ImageSets/480p'

camera_info_folder = physics_info_folder / '480p' / folderIndex
peds_infos_folder = physics_info_folder / '480p' / folderIndex

folders = [rgb_save_folder, depth_save_folder, colored_mask_save_folder,
             objectid_mask_save_folder, imageSets_folder, camera_info_folder,peds_infos_folder]
for f in folders:
    f.mkdir(parents=True, exist_ok=True)

# create necessary names
imageSets_2016 = imageSets_folder / 'val.txt'
imageSets_2017 = imageSets_folder / 'val2017.txt'

# save images and GT for images
for idx,obs in enumerate(observations):
    img_name = '%05i.jpg' % (idx)
    # save image
    rgb_image = Image.fromarray(obs["rgba_camera_1stperson"])
    rgb_image = rgb_image.convert('RGB')
    rgb_file = rgb_save_folder / img_name
    rgb_image.save(str(rgb_file))
    # save depth
    depth_image = Image.fromarray((obs["depth_camera_1stperson"]/ 10 * 255).astype(np.uint8), mode="L")
    depth_file = depth_save_folder / img_name
    depth_image.save(str(depth_file))

    # # save binary mask for moving object
    # binary_image = Image.fromarray((obs["semantic_sensor_1stperson"]*255).astype(np.uint8))
    # binary_file = os.path.join(binary_save_folder, img_name)
    # binary_image.save(binary_file)

    # create RGB image array by mapping grayscale values to RGB values
    gray_image = obs["semantic_sensor_1stperson"]
    shape = gray_image.shape
    objectid_mask_file = objectid_mask_save_folder / img_name
    gray_image = Image.fromarray(gray_image)
    gray_image.convert("L").save(str(objectid_mask_file))
    # also save a colored mask image for better visualization
    colored_mask_image = np.zeros((*shape, 3), dtype=np.uint8)
    for key, value in config["objectid_mapping_id2colors"].items():
        colored_mask_image[np.where(np.array(gray_image) == int(key))] = value
    semantic_colored_file = colored_mask_save_folder / img_name
    colored_mask_image = Image.fromarray(colored_mask_image)
    colored_mask_image.save(str(semantic_colored_file))

    with open(str(imageSets_2016), 'a') as f:
        temp1 = '/JPEGImages/480p/' + folderIndex + '/' + img_name
        temp2 = '/Annotations/480p/' + folderIndex + '/' + img_name
        f.write(temp1 + ' ' + temp2 + '\n')

# save agent/pedestrian movement GT
with open(str(peds_infos_folder / "peds_infos.npy"), 'wb') as f:
    np.save(f, peds_infos)

with open(str(imageSets_2017), 'a') as f:
    f.write(folderIndex + '\n')
with open(str(camera_info_folder / "camera_spec.npy"), 'wb') as f:
    np.save(f, camera_info)
# will oppended
with open(str(output_path / "name_mapping.npy"), 'ab') as f:
    np.save(f, name_mapping)

# video rendering with embedded 1st person view
if make_video:
    video_folder = output_path / 'videos'
    video_folder.mkdir(parents=True, exist_ok=True)
    video_file = video_folder / folderIndex
    # overlay_dims = (int(sim_settings["width"] / 5), int(sim_settings["height"] / 5))
    # print("overlay_dims = " + str(overlay_dims))
    # overlay_settings = [
    #     {
    #          "obs": "semantic_sensor_1stperson",
    #          "type": "semantic",
    #          "dims": overlay_dims,
    #          "pos": (10, 50 + overlay_dims[1] * 2),
    #          "border": 2,
    #      },
    # ]
    # print("overlay_settings = " + str(overlay_settings))

    vut.make_video(
        observations=observations,
        primary_obs="rgba_camera_1stperson",
        primary_obs_type="color",
        video_file=str(video_file),
        fps=int(1.0 / time_step),
        open_vid=show_video,
        # overlay_settings=overlay_settings,
        overlay_settings=None,
        depth_clip=20.0,
    )

# for k,v in sim.__dict__.items():
#     print(k)
#     print(v)

# remove locobot while leaving the agent node for later use
sim.remove_object(locobot_id, delete_object_node=True)
remove_all_objects(sim)
