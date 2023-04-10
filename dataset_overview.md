# HabitatDyn Dataset: Dynamic Object Detection to Kinematics Estimation

***Zhengcheng Shen, Yi Gao, Linh Kästner, Jens Lambrecht***

![data](https://user-images.githubusercontent.com/83227264/230888881-410c0266-9256-4add-a42a-d38bed991be9.jpg)

 We propose a new dataset, HabitatDyn, which includes synthetic RGB videos, semantic labels, and depth information, as well as kinetics information. The dataset features 30 scenes from the perspective of a mobile robot with a moving camera, and contains six different types of moving objects with varying velocities. 

## Abstract
>The aim of the HabitatDyn dataset is to provide a new resource for researchers and developers in the field of mobile robotics that can aid in the creation and advancement of robots with advanced recognition capabilities. The dataset was created to address the limitations of existing image or video processing datasets, which usually do not accurately depict observations from a moving robot and do not contain the kinematics information necessary for robotic tasks. By providing synthetic data that is cost-effective to create and offers greater flexibility for adapting to various applications, the HabitatDyn dataset can be used to train and evaluate algorithms for mobile robots that can navigate and interact with their environments more effectively. Through the use of this dataset, we hope to foster further advancements in the field of mobile robotics and contribute to the development of more capable and intelligent robots.


### Overview

HabitatDyn contains 1590 high-quality videos for 30 meticulously curated scenes from [HM3D](https://aihabitat.org/datasets/hm3d/, citation down below). These scenes feature free, randomly placed object models sourced from the Internet. To add a touch of realism and dynamism, 3 or 6 moving object instances from each model are randomly dropped into the scene, and a robot agent equipped with a camera is also present to capture the scene in detail (camera specifications can be found in the dataset). The videos come with RGB, depth, and semantic annotations, ensuring that researchers have access to a wealth of information for their projects.

Additionally, the objects in each scene have been carefully programmed to move at varying speeds, with 3 levels of movement speed available (1 m/s, 2 m/s, and 3 m/s). To add even more depth to the dataset, each scene has been split into "incl_static" and "excl_static" versions. In the "incl_static" version, 10 static extra object models are randomly placed in the scene, while in the "excl_static" version, only the original objects from the scene and moving instances are present.

Furthermore, the same settings for 3 randomly chosen instances from the 6 models are also generated, making this dataset a comprehensive and versatile resource for researchers in various fields.

### Depth



https://user-images.githubusercontent.com/83227264/230932486-90f0ff66-409c-4d55-a772-6c0d4aad5d0b.mp4



```latex
@inproceedings{ramakrishnan2021hm3d,
  title={Habitat-Matterport 3D Dataset ({HM}3D): 1000 Large-scale 3D Environments for Embodied {AI}},
  author={Santhosh Kumar Ramakrishnan and Aaron Gokaslan and Erik Wijmans and Oleksandr Maksymets and Alexander Clegg and John M Turner and Eric Undersander and Wojciech Galuba and Andrew Westbury and Angel X Chang and Manolis Savva and Yili Zhao and Dhruv Batra},
  booktitle={Thirty-fifth Conference on Neural Information Processing Systems Datasets and Benchmarks Track},
  year={2021},
  url={https://arxiv.org/abs/2109.08238}
}
```

