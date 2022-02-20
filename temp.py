class PedestrianPathFollower(object):
    def __init__(self, sim, semantic_id, agent_scene_node, waypoint_threshold,total_path_number,mls,mts):
        self.curr_path = 0
        self.sim = sim
        self.semantic_id = semantic_id
        self._node = agent_scene_node
        self._threshold = waypoint_threshold
        self.total_path_number = total_path_number
        self._step_size = 0.01
        self.found_paths = []
        self._points = []
        self.progresses = []
        self._lengthes = []
        self._point_progress = []
        self._segment_tangents = []

        # here velocity
        # initial paths to one

    def initial_pedestrian_path(self,obj_template_id,sim):
        obj = rigid_obj_mgr.add_object_by_template_id(obj_template_id)
        obj.semantic_id = self.semantic_id
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, obj.object_id)
        
        continuous_path_followers = []
        # initialize all paths
        for i in range(self.total_path_number):
            self.found_paths.append(habitat_sim.ShortestPath())
        can_sample = sample_object_state(
            sim, obj, from_navmesh=True, maintain_object_up=True, max_tries=1000
        )
        if not can_sample:
            print("Couldn't find an initial object placement. Aborting.")
        self.found_paths[0].requested_start = obj.translation
        # ???
        sim.set_translation(self.found_paths[0].requested_start, obj.object_id)
        self.found_paths[0].requested_end = sim.pathfinder.get_random_navigable_point()
        if self.total_path_number <= 1:
            raise ValueError('error:Path number <= 1')
        for i in range(self.total_path_number - 1):
            self.found_paths[i+1].requested_start = self.found_paths[i].requested_end
            self.found_paths[i+1].requested_end = sim.pathfinder.get_random_navigable_point()
        self.found_paths[-1].requested_end = self.found_paths[0].requested_start
        # find valid initial location and place object nicely

        # if not can_sample:
        #     print("Couldn't find an initial object placement. Aborting.")

        # check if all path navigable: need modification
        found_path = True
        for i in range(self.total_path_number):
            if sim.pathfinder.find_path(self.found_paths[i]):
                continue
            else:
                found_path = False
                print("Could not find path to object, aborting!")
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

        segment_distance = self._length[self.curr_path] * \
            (progress - self._point_progress[self.curr_path][path_ix - 1])
        return (
            self._points[self.curr_path][path_ix - 1]
            + self._segment_tangents[self.curr_path][path_ix - 1] * segment_distance
        )

    def update_waypoint(self):
        if self.progresses[self.curr_path] < 1.0:
            wp_disp = self.waypoint - self._node.translation
            wp_dist = np.linalg.norm(wp_disp)
            node_pos = self._node.translation
            step_size = self._step_size
            threshold = self._threshold
            while wp_dist < threshold:
                self.progresses[self.curr_path] += step_size
                self.waypoint = self.pos_at(self.progresses)
                if self.progresses >= 1.0:
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

    def track_ped_waypoint(vc, mls, mts, dt=1.0 / 60.0):
        angular_error_threshold = 0.5
        max_linear_speed = 1.0
        max_turn_speed = 1.0
        glob_forward = rs.rotation.transform_vector(mn.Vector3(0, 0, -1.0)).normalized()
        glob_right = rs.rotation.transform_vector(mn.Vector3(-1.0, 0, 0)).normalized()
        to_waypoint = mn.Vector3(waypoint) - rs.translation
        u_to_waypoint = to_waypoint.normalized()
        angle_error = float(mn.math.angle(glob_forward, u_to_waypoint))

        new_velocity = 0
        print("Pedestrain angle_error！！！！！: %2.6f" % angle_error)

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
        print("Pedestrain track waypoint ended")
