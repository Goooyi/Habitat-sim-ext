# TODO
velocity 自定义
combine paths for obj to one

# P
1. 30fps / 60fps : It's a coomon issue in simulation world, usually a higher steps per second make the simulation more stable and accurate. Also the collision margin maight help.
2. path3: (s:1.reset oriantation or predefined action or ?)
3. radius = 0.4 won't move
# Habitat-sim-ext
## Idea
* 函数化 and json
* What if agents walk into each other?
* stuck on stairs?
* better navmesh is very important
* do other walker need to do defined walking or just random walk is enough?
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
 * 高度规定，寻路
 * bb限制运动轨迹
 * 找一些复杂的模型（人，机器人）

1.试试新机器人模型
2.台阶高度
3.新的step

## Dataset
[Paper with code](https://paperswithcode.com/sota/unsupervised-video-object-segmentation-on)
1. [DAVIS](https://davischallenge.org/)
2. [KITTI](http://www.cvlibs.net/datasets/kitti/)
3. [LASIESTA](https://www.gti.ssr.upm.es/data/lasiesta_database.html)
4. [PETS2001](https://limu.ait.kyushu-u.ac.jp/dataset/en/)
5. [PESMOD](https://github.com/mribrahim/PESMOD)
6. [LaSOT](https://paperswithcode.com/dataset/lasot)
7. [GOT-10k](https://paperswithcode.com/dataset/got-10k)
8. [TbD-3D](https://paperswithcode.com/dataset/tbd-3d)
9. [VisDrone-Dataset](https://github.com/VisDrone/VisDrone-Dataset)
10. [FMO](http://cmp.felk.cvut.cz/fmo/)

## Paper
https://arxiv.org/abs/2001.05238

https://github.com/tfzhou/MATNet

https://github.com/schmiddo/d2conv3d

https://github.com/sabarim/3DC-Seg

https://github.com/antonilo/unsupervised_detection

https://github.com/antonilo/unsupervised_detection

https://github.com/seoungwugoh/STM

https://www.vision.rwth-aachen.de/page/mots

https://paperswithcode.com/paper/shifting-more-attention-to-video-salient

https://github.com/tri-ml/dd3d

https://github.com/ZSVOS/HGPU

## MOD method
1. [MODNet](http://webdocs.cs.ualberta.ca/~vis/kittimoseg/)

# Issues
1. [Object_id to semantic_id mapping](https://github.com/facebookresearch/habitat-sim/issues/760) and:
https://github.com/facebookresearch/habitat-sim/issues/910
2. [set semantic id](https://github.com/facebookresearch/habitat-sim/pull/668)
3. [use Replica within AI Habitat](https://github.com/facebookresearch/habitat-sim/issues/1256)
