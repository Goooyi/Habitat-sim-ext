# Please refer to branch v0.2.1 for usage
# main branch is only for dev log
# project structure

`temp.py` contains code class/fucntion needed for random walk simulation for Emboded Agent in habitat simulator.
`testwalk.py` is a example to generate video frames using habitat and mp3d dataset.
`schedual.md` just a self dev log

# usage
1. branch v0.2.1 is the version used for this project, install habitat v0.2.1
2. copy `temp.py` and `testwalk.py` to root dictory of habitat
3. run
```
python testwalk.py
```


# HabitatDyn Dataset: Dynamic Object Detection to Kinematics Estimation

***Zhengcheng Shen, Yi Gao, Linh Kästner, Jens Lambrecht***

![data](https://user-images.githubusercontent.com/83227264/230888881-410c0266-9256-4add-a42a-d38bed991be9.jpg)

> We propose a new dataset, HabitatDyn, which includes synthetic RGB videos, semantic labels, and depth information, as well as kinetics information. The dataset features 30 scenes from the perspective of a mobile robot with a moving camera, and contains six different types of moving objects with varying velocities. The aim of the HabitatDyn dataset is to provide a new resource for researchers and developers in the field of mobile robotics that can aid in the creation and advancement of robots with advanced recognition capabilities. The dataset was created to address the limitations of existing image or video processing datasets, which usually do not accurately depict observations from a moving robot and do not contain the kinematics information necessary for robotic tasks. By providing synthetic data that is cost-effective to create and offers greater flexibility for adapting to various applications, the HabitatDyn dataset can be used to train and evaluate algorithms for mobile robots that can navigate and interact with their environments more effectively. Through the use of this dataset, we hope to foster further advancements in the field of mobile robotics and contribute to the development of more capable and intelligent robots.
