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

对预测结果进行uprightness(直立度)，elevation(高程)， flatness(平面度)指标计算。以下3个指标的乘积

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

主要主要部分先看node/offline_kitti.cpp


bairui@DESKTOP-TCT7SII:~/program/patchwork_ws/src/patchwork/launch$ roslaunch patchwork offline_kitti.launch
... logging to /home/bairui/.ros/log/8a850d8a-18b4-11ee-a9b6-59902c089f0f/roslaunch-DESKTOP-TCT7SII-3839.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://DESKTOP-TCT7SII:46691/

SUMMARY
========

PARAMETERS
 * /algorithm: patchwork
 * /data_path: /mnt/d/bairui/brf...
 * /end_frame: 4531
 * /extrinsic_rot: [1, 0, 0, 0, 1, 0...
 * /extrinsic_trans: [0.0, 0.0, 0.0]
 * /patchwork/ATAT/ATAT_ON: True
 * /patchwork/ATAT/max_r_for_ATAT: 5.0
 * /patchwork/ATAT/noise_bound: 0.2
 * /patchwork/ATAT/num_sectors_for_ATAT: 20
 * /patchwork/adaptive_seed_selection_margin: -1.1
 * /patchwork/czm/elevation_thresholds: [0.523, 0.746, 0....
 * /patchwork/czm/flatness_thresholds: [0.0005, 0.000725...
 * /patchwork/czm/min_ranges_each_zone: [2.7, 12.3625, 22...
 * /patchwork/czm/num_rings_each_zone: [2, 4, 4, 4]
 * /patchwork/czm/num_sectors_each_zone: [16, 32, 54, 32]
 * /patchwork/czm/num_zones: 4
 * /patchwork/global_elevation_threshold: -0.5
 * /patchwork/max_r: 80.0
 * /patchwork/min_r: 2.7
 * /patchwork/mode: czm
 * /patchwork/num_iter: 3
 * /patchwork/num_lpr: 20
 * /patchwork/num_min_pts: 10
 * /patchwork/th_dist: 0.125
 * /patchwork/th_seeds: 0.5
 * /patchwork/uniform/num_rings: 16
 * /patchwork/uniform/num_sectors: 54
 * /patchwork/uprightness_thr: 0.707
 * /patchwork/using_global_elevation: False
 * /patchwork/verbose: True
 * /patchwork/visualize: True
 * /rosdistro: noetic
 * /rosversion: 1.16.0
 * /save_flag: False
 * /sensor_height: 1.723
 * /start_frame: 4390
 * /use_sor_before_save: False

NODES
  /
    offline_kitti_DESKTOP_TCT7SII_3839_2541184660714628809 (patchwork/offline_kitti)
    rviz (rviz/rviz)

auto-starting new master
process[master]: started with pid [3847]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 8a850d8a-18b4-11ee-a9b6-59902c089f0f
process[rosout-1]: started with pid [3857]
started core service [/rosout]
process[offline_kitti_DESKTOP_TCT7SII_3839_2541184660714628809-2]: started with pid [3864]
process[rviz-3]: started with pid [3865]
/mnt/d/bairui/brfile/dataset/kitti_odometry_data/dataset/sequences/00/patchwork
[ INFO] [1688287420.582783411]: Inititalizing PatchWork...
Global thr. is not in use
[ INFO] [1688287420.590999034]: Sensor Height: 1.723000
[ INFO] [1688287420.591037078]: Num of Iteration: 3
[ INFO] [1688287420.591051886]: Num of LPR: 20
[ INFO] [1688287420.591056645]: Num of min. points: 10
[ INFO] [1688287420.591080251]: Seeds Threshold: 0.500000
[ INFO] [1688287420.591096312]: Distance Threshold: 0.125000
[ INFO] [1688287420.591122062]: Max. range:: 80.000000
[ INFO] [1688287420.591144695]: Min. range:: 2.700000
[ INFO] [1688287420.591176216]: Num. rings: 16
[ INFO] [1688287420.591199751]: Num. sectors: 54
[ INFO] [1688287420.591207687]: adaptive_seed_selection_margin: -1.100000
[ INFO] [1688287420.592633808]: Uprightness threshold: 0.707000
[ INFO] [1688287420.592711629]: Elevation thresholds: 0.523000 0.746000 0.879000 1.125000
[ INFO] [1688287420.592722540]: Flatness thresholds: 0.000500 0.000725 0.001000 0.001000
[ INFO] [1688287420.592729283]: Num. zones: 4
INITIALIZATION COMPLETE
Target data: /mnt/d/bairui/brfile/dataset/kitti_odometry_data/dataset/sequences/00
Total 4541 files are loaded
4390th node come
Operating patchwork...
[ATAT] The sensor height is auto-calibrated via the ground points in the vicinity of the vehicle
[ATAT] Elevation of the ground w.r.t. the origin is -1.80045 m
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.8805
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.933964
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.809963
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.245881
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.653252
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.00551152
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < 0.180265
4390th,  takes : 0.0185709 | 125883 -> 65936
 P: 95.983 | R: 97.8682
4391th node come
Operating patchwork...
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.990845
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.896369
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.958216
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.778562
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.917277
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.383513
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.340128
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.257407
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.600639
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < 0.498076
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.211398
4391th,  takes : 0.0129337 | 126047 -> 66166
 P: 96.7848 | R: 97.997
4392th node come
Operating patchwork...
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -1.01822
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.930802
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.982111
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.845766
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.870446
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.766523
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.806332
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.169969
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.562176
4392th,  takes : 0.0145104 | 126258 -> 66655
 P: 96.9177 | R: 97.5083
4393th node come
Operating patchwork...
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.834547
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.982643
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.923897
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.863346
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.208551
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.158848
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.549187
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.0636977
[Elevation] Rejection operated. Check 3th param. of elevation_thr_: -0.67545 < -0.112025
4393th,  takes : 0.0142503 | 126187 -> 67999
 P: 96.5317 | R: 97.8323
4394th node come
Operating patchwork...
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.804482
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.99653
[Elevation] Rejection operated. Check 1th param. of elevation_thr_: -1.05445 < -0.933367
[Elevation] Rejection operated. Check 2th param. of elevation_thr_: -0.92145 < -0.42435
4394th,  takes : 0.0132065 | 126058 -> 68610
 P: 96.5863 | R: 97.7964



用来保存Zone的其实是一个二维vector
patches[j][i]表示当前zone中第j个ring中的第i个sector



**什么是LPR？**

covariance matrix是什么？
r7000 电脑为何显示一段时间后卡死，是数据集用用光了吗, 点云数量太小程序提前结束

`roslaunch patchwork offline_kitti.launch`

算法核心在include里<br>
include/tools/kitti_loader.hpp 中定义了将kitti xxx.bin文件读取为点云pcl点云格式

roslaunch文件中
<rosparam command="load" file="$(find patchwork)/config/params.yaml" />也会对参数进行设置


通过rostopic进行点云输入，话题映射在roslaunch中修改
```
roslaunch patchwork run_patchwork.launch is_kitti:=false
```
