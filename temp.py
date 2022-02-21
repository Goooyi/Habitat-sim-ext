from functools import update_wrapper
from turtle import update
import habitat_sim
import magnum as mn
import numpy as np

class PedestrianPathFollower(object):
    def __init__(self, sim, semantic_id, waypoint_threshold,total_path_number,mls,mts,rigid_obj_mgr):
        self.curr_path = 0
        self.sim = sim
        self.semantic_id = semantic_id
        self._threshold = waypoint_threshold
        self.total_path_number = total_path_number
        self._step_size = 0.01
        self.found_paths = []
        self._points = []
        self.progresses = []
        self._lengthes = []
        self._point_progress = []
        self._segment_tangents = []
        self.rigid_obj_mgr = rigid_obj_mgr

        # here velocity
        # initial paths to one

    def initial_pedestrian_path(self,obj_template_id):
        self.obj = self.rigid_obj_mgr.add_object_by_template_id(obj_template_id)
        self.obj.semantic_id = self.semantic_id
        self.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, self.obj.object_id)
        
        # velocity control of the object
        self.vel_control = self.obj.velocity_control
        self.vel_control.controlling_lin_vel = True
        self.vel_control.controlling_ang_vel = True
        self.vel_control.lin_vel_is_local = True
        self.vel_control.ang_vel_is_local = True
        # continuous_path_followers = []
        # initialize all paths
        for i in range(self.total_path_number):
            self.found_paths.append(habitat_sim.ShortestPath())
        # can_sample = sample_object_state(
        #     self.sim, self.obj, from_navmesh=True, maintain_object_up=True, max_tries=1000
        # )
        # if not can_sample:
        #     print("Couldn't find an initial object placement. Aborting.")
        #     raise RuntimeError("Couldn't find an initial object placement. Aborting.")

        # find valid initial location and place object nicely
        
        # check if all path navigable: need modification
        for i in range(self.total_path_number):
            print(self.found_paths[i].requested_start)
            print(self.found_paths[i].requested_end)

        found_path = False
        while not found_path:
            # todo: initial pos
            self.sim.set_translation(self.sim.pathfinder.get_random_navigable_point(), self.obj.object_id)
            self.found_paths[0].requested_start = self.obj.translation
            self.found_paths[0].requested_end = self.sim.pathfinder.get_random_navigable_point()
            if self.total_path_number <= 1:
                raise ValueError('error:Path number <= 1')
            for i in range(self.total_path_number - 1):
                self.found_paths[i+1].requested_start = mn.Vector3(self.found_paths[i].requested_end)
                self.found_paths[i+1].requested_end = mn.Vector3(self.sim.pathfinder.get_random_navigable_point())
            self.found_paths[-1].requested_end = mn.Vector3(self.found_paths[0].requested_start)

            temp = [self.sim.pathfinder.find_path(path) for path in self.found_paths] 
            if sum(temp) == self.total_path_number:
                found_path = True
        # for i in range(self.total_path_number):
        #     if self.sim.pathfinder.find_path(self.found_paths[i]):
        #         continue
        #     else:
        #         print(i)
        #         found_path = False
        #         print("Could not find path to object, aborting!")
        #         raise RuntimeError("Could not find path to object, aborting!")
        # print(len(self.found_paths[0].points))
        self.waypoint = self.found_paths[0].points[0]
        for i in range(self.total_path_number):
            # continuous_path_followers.append(ContinuousPathFollower(sim, found_paths[i], obj, waypoint_threshold=0.4))
            self._points.append(self.found_paths[i].points[:])
            assert len(self._points[i]) > 0
            self.progresses.append(0) # geodesic distance -> [0,1]
            self._lengthes.append(self.found_paths[i].geodesic_distance)
            # setup progress waypoints
            _point_progress = [0]
            _segment_tangents = []
            _length = self._lengthes[i]
            for ix, point in enumerate(self._points[i]):
                if ix > 0:
                    segment = point - self._points[i][ix - 1]
                    segment_length = np.linalg.norm(segment)
                    segment_tangent = segment / segment_length
                    _point_progress.append(
                        segment_length / _length + _point_progress[ix - 1]
                    )
                    # t-1 -> t
                    _segment_tangents.append(segment_tangent)
            self._point_progress.append(_point_progress)
            self._segment_tangents.append(_segment_tangents)
            # final tangent is duplicated
            self._segment_tangents[i].append(self._segment_tangents[i][-1])

    def pos_at(self, progress):
        if progress <= 0:
            return self._points[self.curr_path][0]
        elif progress >= 1.0:
            return self._points[self.curr_path][-1]

        path_ix = 0
        for ix, prog in enumerate(self._point_progress[self.curr_path]):
            if prog > progress:
                path_ix = ix
                break

        segment_distance = self._lengthes[self.curr_path] * \
            (progress - self._point_progress[self.curr_path][path_ix - 1])
        return (
            self._points[self.curr_path][path_ix - 1]
            + self._segment_tangents[self.curr_path][path_ix - 1] * segment_distance
        )

    def update_waypoint(self):
        if self.progresses[self.curr_path] < 1.0:
            wp_disp = self.waypoint - self.obj.translation
            wp_dist = np.linalg.norm(wp_disp)
            node_pos = self.obj.translation
            step_size = self._step_size
            threshold = self._threshold
            while wp_dist < threshold:
                self.progresses[self.curr_path] += step_size
                self.waypoint = self.pos_at(self.progresses[self.curr_path])
                if self.progresses[self.curr_path] >= 1.0:
                    self.progresses[self.curr_path] = 0
                    if self.curr_path == self.total_path_number - 1:
                        self.curr_path = 0
                    else:
                        self.curr_path += 1
                    break
                wp_disp = self.waypoint - node_pos
                wp_dist = np.linalg.norm(wp_disp)
        else:
            self.progresses[self.curr_path] = 0
            if self.curr_path == self.total_path_number - 1:
                self.curr_path = 0
            else:
                self.curr_path += 1

    def track_ped_waypoint(self,mls, mts, dt=1.0 / 60.0):
        angular_error_threshold = 0.5
        max_linear_speed = 1.0
        max_turn_speed = 1.0
        glob_forward = self.obj.rotation.transform_vector(mn.Vector3(0, 0, -1.0)).normalized()
        glob_right = self.obj.rotation.transform_vector(mn.Vector3(-1.0, 0, 0)).normalized()
        to_waypoint = mn.Vector3(self.waypoint) - self.obj.translation
        u_to_waypoint = to_waypoint.normalized()
        angle_error = float(mn.math.angle(glob_forward, u_to_waypoint))

        new_velocity = 0
        print("Pedestrain angle_error！！！！！: %2.6f" % angle_error)

        if angle_error < angular_error_threshold:
            # speed up to max
            new_velocity = (self.vel_control.linear_velocity[2] - max_linear_speed) / 2.0
        else:
            # slow down to 0
            new_velocity = (self.vel_control.linear_velocity[2]) / 2.0
            # new_velocity = 0
        self.vel_control.linear_velocity = mn.Vector3(0, 0, new_velocity)

        # angular part
        rot_dir = 1.0
        
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

        self.vel_control.angular_velocity = mn.Vector3(
            0, np.clip(rot_dir * angular_correction, -max_turn_speed, max_turn_speed), 0
        )
        print("Pedestrain track waypoint ended")

    def integrate_state(self,time_step):
        self.update_waypoint()
        self.track_ped_waypoint(1.0,1.0,time_step)
        previous_rigid_state = self.sim.get_rigid_state(self.obj.object_id)

        target_rigid_state = self.vel_control.integrate_transform(
            time_step, previous_rigid_state
        )
        

        end_pos = self.sim.step_filter(
            previous_rigid_state.translation, target_rigid_state.translation
        )
        self.sim.set_translation(end_pos, self.obj.object_id)
        self.sim.set_rotation(target_rigid_state.rotation, self.obj.object_id)
