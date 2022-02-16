# %%
# Import necessary modules, define some convenience functions, and initialize the Simulator and Agent.
import math
import os
import random
import sys
import git
import magnum as mn
import numpy as np
import habitat_sim
import habitat_sim.physics

from habitat_sim.utils import common as ut
from habitat_sim.utils import viz_utils as vut
from matplotlib import pyplot as plt
from PIL import Image

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
output_path = os.path.join(
    dir_path, "bt_output/"
)
if not os.path.exists(output_path):
    os.mkdir(output_path)

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
        color_sensor_1st_person_spec.postition = [0.0,settings["sensor_height"], 0.0]
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
        depth_sensor_1st_person_spec.postition = [0.0, settings["sensor_height"], 0.0]
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
        semantic_sensor_1st_person_spec.postition = [
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
        color_sensor_3rd_person_spec.postition = [
            0.0,
            settings["sensor_height"] + 0.2,
            0.2,
        ]
        color_sensor_3rd_person_spec.orientation = [-math.pi / 4, 0, 0]
        color_sensor_3rd_person_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(color_sensor_3rd_person_spec)

    # Here you can specify the amount of displacement in a forward action and the turn angle
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs
    return habitat_sim.Configuration(sim_cfg, [agent_cfg])
    

def make_default_settings():
    settings = {
        "width": 720,  # Spatial resolution of the observations
        "height": 544,
        "scene": "./data/scene_datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.glb",  # Scene path
        "default_agent": 0,
        "sensor_height": 1.5,  # Height of sensors in meters
        "sensor_pitch": -math.pi / 8.0,  # sensor pitch (x rotation in rads)
        "color_sensor_1st_person": False,  # RGB sensor
        "color_sensor_3rd_person": True,  # RGB sensor 3rd person
        "depth_sensor_1st_person": False,  # Depth sensor
        "semantic_sensor_1st_person": False,  # Semantic sensor
        "seed": 5,
        "enable_physics": True,  # enable dynamics simulation
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
    # obj_attr_mgr.load_configs(str(os.path.join(data_path, "objects/example_objects")))
    obj_attr_mgr.load_configs(str(os.path.join(data_path, "objects/locobot_merged")))
    prim_attr_mgr = sim.get_asset_template_manager()
    stage_attr_mgr = sim.get_stage_template_manager()
    # Manager providing access to rigid objects
    rigid_obj_mgr = sim.get_rigid_object_manager()

# Define Simulation Utility Functions
def remove_all_objects(sim):
    for obj_id in sim.get_existing_object_ids():
        sim.remove_object(obj_id)


def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())
    return observations

# Set an object transform relative to the agent state
def set_object_state_from_agent(
    sim,
    ob_id,
    offset=np.array([0, 2.0, -1.5]),
    orientation=mn.Quaternion(((0, 0, 0), 1)),
):
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    ob_translation = agent_transform.transform_point(offset)
    sim.set_translation(ob_translation, ob_id)
    sim.set_rotation(orientation, ob_id)


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
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.set_defaults(show_video=False, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.display
    display = args.display
    make_video = args.make_video
else:
    show_video = False
    make_video = False
    display = False


















#%%
# Continuous Path Follower Example
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

        print("self._length = " + str(self._length))
        print("num points = " + str(len(self._points)))
        print("self._point_progress = " + str(self._point_progress))
        print("self._segment_tangents = " + str(self._segment_tangents))

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
    max_linear_speed = 1.0
    max_turn_speed = 1.0
    glob_forward = rs.rotation.transform_vector(mn.Vector3(0, 0, -1.0)).normalized()
    glob_right = rs.rotation.transform_vector(mn.Vector3(-1.0, 0, 0)).normalized()
    to_waypoint = mn.Vector3(waypoint) - rs.translation
    u_to_waypoint = to_waypoint.normalized()
    angle_error = float(mn.math.angle(glob_forward, u_to_waypoint))

    new_velocity = 0
    print("angle_error！！！！！: %2.6f" % angle_error)

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
    
    print(glob_right) ##
    print(u_to_waypoint) ##
    if mn.math.dot(u_to_waypoint,glob_right) < 0:
    # if mn.math.cross(u_to_waypoint,glob_forward)[1] >0  :
        rot_dir = -1.0
    print(rot_dir) ##
    print(mn.math.angle(glob_forward, u_to_waypoint))
    print(mn.math.cross(u_to_waypoint,glob_forward))
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

























#%%
# Embodied Continuous Navigation to find a sampled object
sim_settings = make_default_settings()
# fmt: off
# sim_settings["scene"] = "./data/scene_datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.glb"  # @param{type:"string"}
sim_settings["scene"] = "./data/scene_datasets/habitat-test-scenes/apartment_1.glb"  # @param{type:"string"}
# fmt: on
sim_settings["sensor_pitch"] = 0
sim_settings["sensor_height"] = 0.6
sim_settings["color_sensor_3rd_person"] = True
sim_settings["color_sensor_1st_person"] = True
sim_settings["depth_sensor_1st_person"] = True
sim_settings["semantic_sensor_1st_person"] = True

make_simulator_from_settings(sim_settings)

# navmesh settings
default_nav_mesh_settings = habitat_sim.NavMeshSettings()
default_nav_mesh_settings.set_defaults()
inflated_nav_mesh_settings = habitat_sim.NavMeshSettings()
inflated_nav_mesh_settings.set_defaults()
inflated_nav_mesh_settings.agent_radius = 0.2
inflated_nav_mesh_settings.agent_height = 1.5
inflated_nav_mesh_settings.agent_max_climb = 10
inflated_nav_mesh_settings.agent_max_slope = 60

# sim.config.sim_cfg.allow_sliding = True
# recompute_successful = sim.recompute_navmesh(sim.pathfinder, inflated_nav_mesh_settings)
# if not recompute_successful:
#     print("Failed to recompute navmesh!")

# Set other example parameters:


seed = 4  # @param {type:"integer"}
random.seed(seed)
sim.seed(seed)
np.random.seed(seed)
# @param {type:"boolean"}
# Load the selected object, here we use locobot and place it on the NavMesh
locobot_template_id = obj_attr_mgr.load_configs(
    str(os.path.join(data_path, "objects/locobot_merged")),False
)[0]
# load a selected target object and place it on the NavMesh
# obj_id_1 = sim.add_object(locobot_template_id)


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

# reset observations and robot state
# locobot_obj.translation = sim.pathfinder.get_random_navigable_point()
sim.set_translation(sim.pathfinder.get_random_navigable_point(), locobot_id)
observations = []


_locobot_01 = rigid_obj_mgr.add_object_by_template_id(locobot_template_id)
_locobot_02 = rigid_obj_mgr.add_object_by_template_id(locobot_template_id)
_locobot_03 = rigid_obj_mgr.add_object_by_template_id(locobot_template_id)
_locobot_04 = rigid_obj_mgr.add_object_by_template_id(locobot_template_id)
sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, _locobot_01.object_id)
sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, _locobot_02.object_id)
sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, _locobot_03.object_id)
sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, _locobot_04.object_id)
sim.set_translation(sim.pathfinder.get_random_navigable_point(), _locobot_01.object_id)
sim.set_translation(sim.pathfinder.get_random_navigable_point(), _locobot_02.object_id)
sim.set_translation(sim.pathfinder.get_random_navigable_point(), _locobot_03.object_id)
sim.set_translation(sim.pathfinder.get_random_navigable_point(), _locobot_04.object_id)

_locobot_01.semantic_id = 1
_locobot_02.semantic_id = 2
_locobot_03.semantic_id = 3
_locobot_04.semantic_id = 4





# get shortest path to the object from the agent position
found_path = False
path1 = habitat_sim.ShortestPath()
path2 = habitat_sim.ShortestPath()
path3 = habitat_sim.ShortestPath()
path4 = habitat_sim.ShortestPath()
path5 = habitat_sim.ShortestPath()
while not found_path:
    # can_sample,destination = sample_object_state(
    #     sim, obj_id_1, from_navmesh=True, maintain_object_up=True, max_tries=1000
    # )
    # if not can_sample:
    #     print("Couldn't find an initial object placement. Aborting.")
    #     break
    # destination = sim.pathfinder.get_random_navigable_point()

    # path1.requested_start = locobot_obj.translation
    # path1.requested_end = obj_1.translation
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
    # found_path = sim.pathfinder.find_path(path1)
    # found_path = sim.pathfinder.find_path(path1) and sim.pathfinder.find_path(path2)
# sim.remove_object(obj_id_1)

if not found_path:
    print("Could not find path to object, aborting!")

vis_objs = []

recompute_successful = sim.recompute_navmesh(sim.pathfinder, default_nav_mesh_settings)
if not recompute_successful:
    print("Failed to recompute navmesh 2!")



continuous_path_follower = ContinuousPathFollower(
    sim, path1, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.4
)

navmesh_settings = habitat_sim.NavMeshSettings()


# set fps
show_waypoint_indicators = True  # @param {type:"boolean"}
time_step = 1.0 / 30.0

sim.config.sim_cfg.allow_sliding = True

for i in range(5):
    if i == 1:
        print("@111111111111@")
        # path2.requested_start = mn.Vector3(sim.get_translation(locobot_id))
        # sim.pathfinder.find_path(path2)
        print(sim.get_translation(locobot_id))
        continuous_path_follower = ContinuousPathFollower(
            sim, path2, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
        )
    if i == 2:
        print("$222222222222$")
        print(sim.get_translation(locobot_id))
        continuous_path_follower = ContinuousPathFollower(
            sim, path3, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
        )

    if i == 3:
        print("$333333333$")
        print(sim.get_translation(locobot_id))
        continuous_path_follower = ContinuousPathFollower(
            sim, path4, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
        )

    if i == 4:
        print("$444444444$")
        print(sim.get_translation(locobot_id))
        continuous_path_follower = ContinuousPathFollower(
            sim, path5, sim.get_object_scene_node(locobot_id), waypoint_threshold=0.8
        )


    if show_waypoint_indicators:
        for vis_obj in vis_objs:
            # rigid_obj_mgr.remove_object_by_id(vis_obj.object_id)
            sim.remove_object(vis_obj)
        vis_objs = setup_path_visualization(sim,continuous_path_follower)

    # manually control the object's kinematic state via velocity integration
    start_time = sim.get_world_time()
    max_time = 20.0
    previous_pos = sim.get_translation(locobot_id)
    while (
        continuous_path_follower.progress < 1.0
        and sim.get_world_time() - start_time < max_time
    ):
        continuous_path_follower.update_waypoint()
        if show_waypoint_indicators:
            #  vis_objs[0].translation = continuous_path_follower.waypoint
            sim.set_translation(continuous_path_follower.waypoint, vis_objs[0])

        if locobot_id < 0:
            print("locobot_id " + str(locobot_id.object_id))
            break

        previous_rigid_state = sim.get_rigid_state(locobot_id)

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

        # sim.set_translation(end_pos, locobot_id)
        sim.set_translation(target_rigid_state.translation, locobot_id)
        print("rotation here!!!rotation here!!!rotation here!!!")
        print(target_rigid_state.rotation)
        sim.set_rotation(target_rigid_state.rotation, locobot_id)

        # Check if a collision occured
        dist_moved_before_filter = (
            target_rigid_state.translation - previous_rigid_state.translation
        ).dot()
        dist_moved_after_filter = (end_pos - previous_rigid_state.translation).dot()

        print(i)
        print(continuous_path_follower.progress)
        print(sim.get_translation(locobot_id))
        print("Path1 requested End")
        print(mn.Vector3(path1.points[-1]))
        print("Path2 requested End")
        print(mn.Vector3(path2.requested_end))
        print(mn.Vector3(path2.points[-1]))
        print("Path3 requested End")
        print(mn.Vector3(path3.requested_end))
        print(mn.Vector3(path3.points[-1]))
        

        # if continuous_path_follower.progress < 1.0:
        #     sim.set_translation(end_pos, locobot_id)

        # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
        # collision _didn't_ happen. One such case is going up stairs.  Instead,
        # we check to see if the the amount moved after the application of the filter
        # is _less_ than the amount moved before the application of the filter
        EPS = 1e-5
        # collided = (dist_moved_after_filter) < dist_moved_before_filter or (previous_pos-sim.get_translation(locobot_id)).dot() < EPS
        collided =  (previous_pos-sim.get_translation(locobot_id)).dot() < EPS
        if(collided):
            print("Warning: collided here!!")
        #     # TODO: reset or change oriantation so it can continue to run?
        #     if(i == 0):
        #         print("!!!!!!!!!!!!collision during first Path!!!!!!!!!!!!")
        #     if(i == 0 and continuous_path_follower.progress >= 1.0):
        #         print("!!!!!!!!!!!!Finished first path!!!!!!!!!!!!")
        #     print("******************************************************************************")
        #     if(i == 1 and continuous_path_follower.progress >= 1.0):
        #         print("!!!!!!!!!!!!Finished second path!!!!!!!!!!!!")
        #     if(i == 1):
        #         print(continuous_path_follower.progress)
        #         print("!!!!!!!!!!!!collision during second Path!!!!!!!!!!!!")
        #     print(sim.get_world_time() - start_time)
        #     if((path1.requested_end == sim.get_translation(locobot_id)).all()):
        #         print("yes,yes,yes,yes,yes,yes,yes,yes,")
        #     break

        previous_pos = sim.get_translation(locobot_id)
        
        # run any dynamics simulation
        sim.step_physics(time_step)

        # render observation
        observations.append(sim.get_sensor_observations())
# release
start_time = sim.get_world_time()
while sim.get_world_time() - start_time < 2.0:
    sim.step_physics(time_step)
    observations.append(sim.get_sensor_observations())



# video rendering with embedded 1st person view
video_prefix = "test_walk"
if make_video:
    overlay_dims = (int(sim_settings["width"] / 5), int(sim_settings["height"] / 5))
    print("overlay_dims = " + str(overlay_dims))
    overlay_settings = [
        # {
        #     "obs": "rgba_camera_1stperson",
        #     "type": "color",
        #     "dims": overlay_dims,
        #     "pos": (10, 10),
        #     "border": 2,
        # },
        # {
        #     "obs": "depth_camera_1stperson",
        #     "type": "depth",
        #     "dims": overlay_dims,
        #     "pos": (10, 30 + overlay_dims[1]),
        #     "border": 2,
        # },
        {
            "obs": "semantic_sensor_1stperson",
            "type": "semantic",
            "dims": overlay_dims,
            "pos": (10, 50 + overlay_dims[1] * 2),
            "border": 2,
        },
    ]
    print("overlay_settings = " + str(overlay_settings))

    vut.make_video(
        observations=observations,
        primary_obs="rgba_camera_3rdperson",
        primary_obs_type="color",
        video_file=output_path + video_prefix,
        fps=int(1.0 / time_step),
        open_vid=show_video,
        overlay_settings=overlay_settings,
        depth_clip=20.0,
    )

objs = sim.semantic_scene.objects
print(objs)
for obj in objs:
    print(obj.semantic_id)

# remove locobot while leaving the agent node for later use
# rigid_obj_mgr.remove_object_by_id(locobot_obj.object_id, delete_object_node=False)
# rigid_obj_mgr.remove_all_objects()

sim.remove_object(locobot_id, delete_object_node=True)
remove_all_objects(sim)
