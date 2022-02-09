class PedestrianPathFollower(object):
    
    curr_continuous_path_follower = None

    def __init__(self, sim, path, agent_scene_node, waypoint_threshold):
        temp = 0

    def pedestrian(obj_template_id,sim,total_path_number,semantic_id):
        obj = rigid_obj_mgr.add_object_by_template_id(obj_template_id)
        obj.semantic_id = 1
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, obj.object_id)
        finded_paths = []
        continuous_path_followers = []
        # initialize all paths
        for i in range(total_path_number):
            finded_paths.append(habitat_sim.ShortestPath())
        can_sample = sample_object_state(
            sim, obj, from_navmesh=True, maintain_object_up=True, max_tries=1000
        )
        if not can_sample:
            print("Couldn't find an initial object placement. Aborting.")
        finded_paths[0].requested_start = obj.translation
        sim.set_translation(finded_paths[0].requested_start, obj.object_id)
        finded_paths[0].requested_end = sim.pathfinder.get_random_navigable_point()
        for i in range(total_path_number - 1):
            finded_paths[i+1].requested_start = finded_paths[i].requested_end
            finded_paths[i+1].requested_end = sim.pathfinder.get_random_navigable_point()
        finded_paths[-1].requested_end = finded_paths[0].requested_start
        # find valid initial location and place object nicely

        if not can_sample:
            print("Couldn't find an initial object placement. Aborting.")

        # check if all path navigable
        found_path = True
        for i in range(total_path_number):
            if sim.pathfinder.find_path(finded_paths[i]):
                continue
            else:
                found_path = False
                print("Could not find path to object, aborting!")
        for i in range(total_path_number):
            continuous_path_followers.append(ContinuousPathFollower(sim, finded_paths[i], obj, waypoint_threshold=0.4))
