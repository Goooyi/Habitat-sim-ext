class PedestrianPathFollower(object):
    def __init__(self, sim, semantic_id, agent_scene_node, waypoint_threshold,total_path_number):
        self.curr_continuous_path_follower = 0
        self.sim = sim
        self.semantic_id = semantic_id
        self._node = agent_scene_node
        self._threshold = waypoint_threshold
        self.total_path_number = total_path_number
        self._step_size = 0.01
        self.found_paths = []
        self.waypoints = []
        self.progresses = []
        self._lengthes = []

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
        sim.set_translation(self.found_paths[0].requested_start, obj.object_id)
        self.found_paths[0].requested_end = sim.pathfinder.get_random_navigable_point()
        for i in range(total_path_number - 1):
            self.found_paths[i+1].requested_start = self.found_paths[i].requested_end
            self.found_paths[i+1].requested_end = sim.pathfinder.get_random_navigable_point()
        self.found_paths[-1].requested_end = self.found_paths[0].requested_start
        # find valid initial location and place object nicely

        if not can_sample:
            print("Couldn't find an initial object placement. Aborting.")

        # check if all path navigable: need modification
        found_path = True
        for i in range(self.total_path_number):
            if sim.pathfinder.find_path(self.found_paths[i]):
                continue
            else:
                found_path = False
                print("Could not find path to object, aborting!")
        for i in range(self.total_path_number):
            # continuous_path_followers.append(ContinuousPathFollower(sim, found_paths[i], obj, waypoint_threshold=0.4))
            self.waypoints.append(self.found_paths[i].points[0])
            self.progresses.append(0) # geodesic distance -> [0,1]
            self._lengthes.append(self.found_paths[i].geodesic_distance)
    def waypoints_manager(self):

