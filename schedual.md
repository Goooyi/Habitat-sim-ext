 pedestrian():
    load object

    set motion type

    get vel_contro

    set agent location

    set semantic id

    set how many paths to acquire and set each path's start and end, make them go back


path finding():
    do found_path, which get a list a navigable path point

    get a continuous_path_follower class object for each path

do path walking():
    while continuous_path_follower.progress >= 1 go to next path

    if it's last pahh, go to the first one.

track_waypoint_to_get_vel():
    call track_waypoint() function to set velocities

update pedestrian state():
    manually integrate the rigid state: set_translation() and set_rotation()

!!!!!!!here!!!!!:
if all paths has been through, go back again
