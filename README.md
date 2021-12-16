# P
1. 30fps / 60fps : It's a coomon issue in simulation world, usually a higher steps per second make the simulation more stable and accurate. Also the collision margin maight help.
2. path3: (s:1.reset oriantation or predefined action or ?)
3. radius = 0.4 won't move
# Habitat-sim-ext
## Idea
* 函数化
```
from habitat_sim.utils import mypy
##############
is_success, location?? = mypy.do_someth(sim, obj_id, jason_and_object_path = 'data/objects')
```
then in mypy.py
```
def mypy(sim, obj_id, scale, jason_and_object_path):
  config = loadjason(jason_and_object_path)
  
  name = config["name"]
  kinematic = config["kinematic"]
  motion_type = ...DYNAMIC? KINEMATIC? STATIC?
  # if DYNAMIC:
  forces,torques,instantanious_initial_velocities
  # if KINEMATIC:
  translation,rotation
  
  velocity = ...
  obj_template_handle = obj_attr_mgr.get_file_template_handles(name)[0]
 ```
 * Habitat 2.0 Tutorial [Ref](https://aihabitat.org/docs/habitat-sim/managed-rigid-object-tutorial.html#kinematic-object-placement), include: modify an object's user-defined configurations
 ## GeneralP
 * Kinematic复杂的运动?接收一个translation matrix？ 那之后bounding box怎么办？

 * 在用户给定的bb
 * Kinematic will not interact with the scene, what happend when collision with other object or other scence?
 

* 高度规定，寻路
* 初识点赞时random， 暂时一种运动方式，往复循环运动
* 寻路的时候的action
* bb限制运动轨迹
* 找一些复杂的模型（人，机器人）

1.试试新机器人模型
2.台阶高度
3.新的step

## Dataset
[Paper with code](https://paperswithcode.com/sota/unsupervised-video-object-segmentation-on)
1. [DAVIS](https://davischallenge.org/)
2. [PETS2001]([https://limu.ait.kyushu-u.ac.jp/dataset/en/)
3. [PESMOD](https://github.com/mribrahim/PESMOD)
4. [LaSOT](https://paperswithcode.com/dataset/lasot)
5. [GOT-10k](https://paperswithcode.com/dataset/got-10k)
6. [TbD-3D](https://paperswithcode.com/dataset/tbd-3d)
7. [VisDrone-Dataset](https://github.com/VisDrone/VisDrone-Dataset)
