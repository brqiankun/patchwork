# Patchwork

Official page of *"Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor"*, which is accepted by RA-L with IROS'21 option 

#### [[Video](https://youtu.be/rclqeDi4gow)] [[Preprint Paper](https://urserver.kaist.ac.kr/publicdata/patchwork/RA_L_21_patchwork_final_submission.pdf)] [[Project Wiki](https://github.com/LimHyungTae/patchwork/wiki)]

Patchwork                  |  Concept of our method (CZM & GLE)
:-------------------------:|:-------------------------:
![](img/patchwork_concept_resized.jpg) |  ![](img/patchwork.gif)

It's an overall updated version of **R-GPF of ERASOR** [**[Code](https://github.com/LimHyungTae/ERASOR)**] [**[Paper](https://arxiv.org/abs/2103.04316)**]. 

## NEWS (22.12.24)
- Merry christmas eve XD! `include/label_generator` is added to make the `.label` file, following the SemanticKITTI format.
- The `.label` files can be directly used in [3DUIS benchmark](https://github.com/PRBonn/3DUIS)
- Thank [Lucas Nunes](https://scholar.google.com/citations?user=PCxhsf4AAAAJ&hl=en&oi=ao) and [Xieyuanli Chen](https://scholar.google.com/citations?user=DvrngV4AAAAJ&hl=en&oi=sra) for providing code snippets to save a `.label` file.

## NEWS (22.07.25)
- Pybinding + more advanced version is now available on [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) as a preprocessing step for deep learning users (i.e., python users can also use our robust ground segmentation)!

## NEWS (22.07.13)
- For increasing convenience of use, the examples and codes are extensively revised by reflecting [issue #12](https://github.com/LimHyungTae/patchwork/issues/12). 

## NEWS (22.05.22)
- The meaning of `elevation_thresholds` is changed to increase the usability. The meaning is explained in [wiki](https://github.com/LimHyungTae/patchwork/wiki/4.-IMPORTANT:-Setting-Parameters-of-Patchwork-in-Your-Own-Env.).
- A novel height estimator, called *All-Terrain Automatic heighT estimator (ATAT)* is added within the patchwork code, which auto-calibrates the sensor height using the ground points in the vicinity of the vehicle/mobile robot. 
  - Please refer to the function `consensus_set_based_height_estimation()`.

## NEWS (21.12.27)
- `pub_for_legoloam` node for the pointcloud in kitti bagfile is added.
	- `ground_estimate.msg` is added
- Bug in xy2theta function is fixed.

- How to run
```bash
roslaunch patchwork pub_for_legoloam.launch
rosbag play {YOUR_FILE_PATH}/KITTI_BAG/kitti_sequence_00.bag --clock /kitti/velo/pointcloud:=/velodyne_points
```
- **This README about this LiDAR odometry is still incomplete. It will be updated soon!**
----

# Demo

## KITTI 00 

![](img/demo_kitti00_v2.gif)

## Rough Terrain

![](img/demo_terrain_v3.gif)

----


### Characteristics

* Single hpp file (`include/patchwork/patchwork.hpp`)

* Robust ground consistency

As shown in the demo videos and below figure, our method shows the most promising robust performance compared with other state-of-the-art methods, especially, our method focuses on the little perturbation of precision/recall as shown in [this figure](img/seq_00_pr_zoom.pdf).

Please kindly note that the concept of *traversable area* and *ground* is quite different! Please refer to our paper.


## Contents
0. [Test Env.](#Test-Env.)
0. [Requirements](#requirements)
0. [How to Run Patchwork](#How-to-Run-Patchwork)
0. [Citation](#citation)

### Test Env.

The code is tested successfully at
* Linux 18.04 LTS
* ROS Melodic

## Requirements

### ROS Setting
- 1. Install [ROS](http://torch.ch/docs/getting-started.html) on a machine. 
需要安装jsk_visual
- 2. Thereafter, [jsk-visualization](https://github.com/jsk-ros-pkg/jsk_visualization) is required to visualize Ground Likelihood Estimation status.

```bash
sudo apt-get install ros-melodic-jsk-recognition
sudo apt-get install ros-melodic-jsk-common-msgs
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

- 3. Compile compile this package. We use [catkin tools](https://catkin-tools.readthedocs.io/en/latest/),
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LimHyungTae/patchwork.git
cd .. && catkin build patchwork 
```

## How to Run Patchwork

We provide four examples:

* How to run Patchwork in SemanticKITTI dataset
    * Offline KITTI dataset
    * Online (ROS Callback) KITTI dataset

* How to run Patchwork in your own dataset
    * Offline by loading pcd files
    * Online (ROS Callback) using your ROS bag file

### Offline KITTI dataset

1. Download [SemanticKITTI](http://www.semantic-kitti.org/dataset.html#download) Odometry dataset (We also need labels since we also open the evaluation code! :)

2. Set the `data_path` in `launch/offline_kitti.launch` for your machine.

The `data_path` consists of `velodyne` folder and `labels` folder as follows:

```
data_path (e.g. 00, 01, ..., or 10)
_____velodyne
     |___000000.bin
     |___000001.bin
     |___000002.bin
     |...
_____labels
     |___000000.label
     |___000001.label
     |___000002.label
     |...
_____...
   
```

3. Run launch file 
```
roslaunch patchwork offline_kitti.launch
```

You can directly feel the speed of Patchwork! :wink:

### Online (ROS Callback) KITTI dataset

We also provide rosbag example. If you run our patchwork via rosbag, please refer to this example.

1. After building this package, run the roslaunch as follows:

```
roslaunch patchwork run_patchwork.launch is_kitti:=true
```

Then you can see the below message:

![](/img/kitti_activated.png)

2. Set the `data_path` in `launch/kitti_publisher.launch` for your machine, which is same with the aforementioned parameter in "Offline KITTI dataset" part. 

3. Then, run ros player (please refer to `nodes/ros_kitti_publisher.cpp`) by following command at another terminal window:
 
```
roslaunch patchwork kitti_publisher.launch
```


### Own dataset using pcd files

Please refer to `/nodes/offilne_own_data.cpp`. 

(Note that in your own data format, there may not exist ground truth labels!)

Be sure to set right params. Otherwise, your results may be wrong as follows:

W/ wrong params            | After setting right params
:-------------------------:|:-------------------------:
![](img/ouster128_wrong_elevation.png) |  ![](img/ouster128_right_elevation.png)

For better understanding of the parameters of Patchwork, please read [our wiki, 4. IMPORTANT: Setting Parameters of Patchwork in Your Own Env.](https://github.com/LimHyungTae/patchwork/wiki/4.-IMPORTANT:-Setting-Parameters-of-Patchwork-in-Your-Own-Env.).


#### Offline (Using *.pcd or *.bin file)

1. Utilize `/nodes/offilne_own_data.cpp`

2. Please check the output by following command and corresponding files:

3. Set appropriate absolute file directory, i.e. `file_dir`, in `offline_ouster128.launch` 
```
roslaunch patchwork offline_ouster128.launch
```

#### Online (via your ROS bag file)

It is easy by re-using `run_patchwork.launch`.

1. Remap the topic of subscriber, i.g. modify remap line as follows:

```
<remap from="/patchwork/cloud" to="$YOUR_LIDAR_TOPIC_NAME$"/>
```

Note that the type subscribed data is `sensor_msgs::PointCloud2`.

2. Next, launch the roslaunch file as follows:

```
roslaunch patchwork run_patchwork.launch is_kitti:=false
```

Note that `is_kitti=false` is important! Because it decides which rviz is opened. The rviz shows only estimated ground and non-ground because your own dataset may have no point-wise labels.

3. Then play your bag file!
 
```
rosbag play $YOUR_BAG_FILE_NAME$.bag
```

## Citation

If you use our code or method in your work, please consider citing the following:

	@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
    }

---------

### Description

All explanations of parameters and other experimental results will be uploaded in [wiki](https://github.com/LimHyungTae/patchwork/wiki)

### Contact

If you have any questions, please let me know:

- Hyungtae Lim {[shapelim@kaist.ac.kr]()}


### TODO List

- [x] Add ROS support
- [x] Add preprint paper
- [x] Add demo videos
- [x] Add own dataset examples
- [x] Update wiki

--------------


## Patchwork
不同的思路, 纯cpp实现，没有深度学习方式，其几何算法是否能更有效的进行特征提取，输入到网络中？
https://github.com/LimHyungTae/patchwork
https://github.com/url-kaist/patchwork-plusplus
[Patchwork++](http://www.guyuehome.com/38829)

地面点云分割主要是:
- 为了解决找到可通行区域(movable area), 
- 还可以用于**识别跟踪物体?**, 分割地面点云可以起到降低计算复杂度(大多数都是地面点云，可以作为**预处理阶段**，先去除地面点云，降低后续计算复杂度)
针对地面的凹凸不平以及斜坡等类别 <br>

分割速度达到40Hz，算法**针对城市环境**

[!patchwork_framework]()
算法结构
1. 点云被编码进Concentric Zone Model-based representation(基于同心圆区域模型表示)，使得点云密度分配均匀？计算复杂度低(指同心圆表示计算)   CZM 表示
2. 之后进行Region-wise Ground Plane Fitting(区域级的地面拟合， R-GPF)， 评估每个区域的地面？
3. Ground Likelihood Estimation(地面似然/可能性估计， GLE)，以减少假阳率  uprightness直立度，elevation高程，flatness平整度
地面包括移动物体可通行的区域，草地，人行道等

- 基于高度过滤以及RANSAC的方法无法分割陡坡，颠簸，不均匀，周围物体影响效果
- 现有地面评估算法时效性问题，不适合预处理
- 扫描表示(点云表示方式？)
- elevation map-based 2.5D grid representation 基于高程图的2.5D地图表示, 用来区分是否属于地面点来将3D点云表示为2.5D格式. 无法表征陡坡，在Z变化较快时？ 到底什么是2.5D
- 深度学习方法在实际应用
时需要，使用环境与训练环境相近(即模型泛化能力)，传感器配置


点云被分为两类地面点云G， 和剩余的所有点的集合Gc


#### CZM
concentric zone model 同心圆模型
假设真实地面可以在小范围内(small parts)是平坦的
针对lidar数据本身近密远疏的特点，坐标系划分存在远距离稀疏性(点云太稀疏无法找到接地层)，近距离存在可表示问题(网格太小)

CZM,给网格分配了合适的密度大小，划分为不同区域，每个区域由不同大小的bin(网格)组成。同时计算不复杂。
在论文中将同心圆划分为4个区域。每个区域包括Nrm * N
最内层区域和最外层区域的网格划分较稀疏，来解决远距离稀疏和近距离可表示的问题，同时减少了bin(网格)的数量

#### R-GPF
Region-wise Ground Plane Fitting 区域级的地面拟合
每个bin通过R-GPF来进行估计，之后合并部分地面点。使用Principal Component Analysis(PCA)主成分分析，相比RANSAC更快(至少2倍)。

C是一个bin中点云的协方差矩阵，计算出C的3个特征值和特征向量。**对应于最小特征值的特征向量是最有可能表示对应于地面层的法向量n**。根据法向量n和单位空间的平均点计算平面系数d。
将高度最低的bin作为地表。
**初始估计到底怎么实现的??**
迭代估计地面点云。第l次迭代得到的**地面(估计)点云的法向量(normal vector)**， 之后计算**平面系数**。
之后迭代计算第l + 1次迭代。
paper中迭代3次？
相比原版R-GPF，使用了自适应的初始seed，防止收敛到局部最小。

**过滤由于Lidar本身缺陷导致的位于真实地面以下的点云。**


#### GLE
Ground Likelihood Estimation
对于二分类，区域级的随机测试，来改善整体预测准确率，排除包含非地面点的初始(unintended planes)

对预测结果进行uprightness(直立度)，elevation(高程)， flatness(平面度)指标计算。

- Uprightness
法向量是否趋向于传感器的XY平面, 法向量与[0, 0, 1]求夹角。设置一个直立裕度(uprightness margin)，相当于根据夹角阈值进行判断
- Elevation
由于直立度无法区分汽车顶，引擎盖等平面和地面的区别；以及当大型物体遮挡时，产生部分观测问题。即部分点云被预测为平面，部分不是，主要是为了去除FP
使用一个条件逻辑函数**conditional logistic function**. 传感器附近高度平均值比-hs高，可能不是地面？  感觉公式有问题，指数分子多一个负号？
对距离中心较近的区域才可以使用该方法
- Flatess
主要是为了恢复可能被Elevation过滤的FN，比如陡坡。设置表面变量阈值，阈值依赖Zm(区域)，通过这样，陡坡的GLE增加。

根据GLE是否大于0.5来对初步预测结果进行过滤(逻辑与？)


kitti数据集中设置lane marking, road, parking, sidewalk, other ground, vegetation, and terrain类别的点为地面真值，植被高度低于-1.3的也被设置为地面点
也在崎岖道路进行试验

论文中kitti数据集算法性能达到43.97Hz ??









