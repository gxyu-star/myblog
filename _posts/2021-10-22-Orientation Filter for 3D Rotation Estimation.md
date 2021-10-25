---
layout: post
title: "Orientation Filter for 3D Rotation Estimation"
author: "gxyu"
---

Orientation Filter的任务是通过融合陀螺仪/加速度计/磁力计的测量数据，估计刚体准确的方向（刚体与地球之间的相对旋转）。

## 1. Mahony算法


## 2. Madgwick算法

   Sebastian O.H. Madgwick在2010年提出了该算法，并公开了paper的内容。在2011年将paper发表在了ICRA上，同时释放了源码。Madgwick算法的核心是基于优化的梯度下降法，将陀螺仪的姿态计算过程中，融合加速度计或者磁力计进行方向较正，以达到姿态解算的结果。算法的创新点主要包括：**根据系统特征提供可调的参数；支持低采样频率下的姿态解算；支持在线磁力计畸变和陀螺仪漂移补偿；此外源码还做了相应的优化以降低CPU负载**。
   算法的系统框图如1所示。图中的红框部分是陀螺仪积分过程，黄色框部分是利用加速度计和磁力计进行补偿的过程。此外，蓝色框和绿色框的两部分分别是磁力计畸变较正和陀螺仪零偏补偿。接下来将根据框图中的每一部分详细讨论一下算法的原理。
   <div align=center>
   <img src="../images/Madgwick%20block%20diagram.png"/>
   </div>   
   
   ### 2.1 陀螺仪积分获取姿态的过程
   
   陀螺仪测量的是基于传感器坐标系表示三个轴的角速度值$(\omega_x, \omega_y, \omega_z)$，为了方便和四元数进行运算，将其转换为一个四位向量：
   
   $$^S\omega_t=[0\ \omega_x\ \omega_y\ \omega_z]_t$$
   
   由当前角速度和旋转，可以求出当前旋转的倒数的四元数形式：

   $$_E^S\dot{q}_{w,t}=\frac{1}{2}*_E^S\hat q_{est,t-1}\otimes^S\omega_t$$

   当前的旋转是通过前一时刻的旋转进行一次积分得到的：

   $$_E^Sq_{w,t}=_E^S\hat q_{est,t-1}+_E^S\dot{q}_{w,t}*\Delta t$$

   这样就把旋转的积分转换成了四元数的线性运算。

   ## 2.2 加速度计或磁力计获取姿态的过程

   陀螺仪的积分是在前一时刻的基础上进行的，在初始状态下，Sensor和Earth的相对关系往往不是单位矩阵，而是有一定的初始角度的。要想拿到初始的旋转就需要加速度和磁力计的帮助，并且还需要一个基本的假设：初始的时候sensor是静止或匀速直线运动，且磁场恒定。仅依靠加速度计或者仅依靠磁力计是无法获得完整的初始旋转的，原因在于：第一，竖直朝下的重力在Yaw方向上是没有分量的；第二，南北指向的磁力，在Roll方向上是没有分量的。这就解释了采用IMU解算出来的旋转在Yaw方向上是有任意初值的。只有MARG Sensor才能有绝对的Yaw指向。

   Madgwick算法采用了最优化的方法进行初始旋转的计算，并采用梯度下降的方法获求出cost function的最小值。可以理解为初始旋转的最优解，是将earth frame的方向转到sensor frame的方向，使得误差最小的那个解。每迭代一次就是用旋转矩阵对earth frame做一次旋转，得到旋转后的方向向量。然后将方向向量和sensor测量的方向做残差，残差最小的时候就说明旋转后的earth frame和sensor frame重叠了。这时候的旋转矩阵就是earth frame和sensor frame之间的相对旋转。具体的公式如下：

   $$\underset{_E^{S}\hat q\in\Re^4}{min}f(_E^{S}\hat q,\ ^{E}\hat d,\ ^{S}\hat s)$$

   $$f(_E^{S}\hat q,\ ^{E}\hat d,\ ^{S}\hat s)=_E^{S}\hat q^*\otimes^{E}\hat d\otimes_E^{S}\hat q-^{S}\hat s$$

   $$_E^{S}\hat q=[qw, qx, qy, qz]$$

   $$^{E}\hat d=[0, dx, dy, dz]$$

   $$^{S}\hat s=[0, sx, sy, sz]$$

   $^{E}\hat d$是预先定义的earth的方向，地磁和重力可以确定一个地球的Global坐标系，$^{E}\hat d$就用来表示这个坐标系的方向。$^{S}\hat s$是传感器的测量值，是外部作用力（磁力或者加速度）在Sensor坐标系下的值。**这个方程表达了将Global的方向转到Sensor坐标系下，求两个方向之间的差值。** 求解旋转的过程，就是不断缩小这个差值。每一步迭代就是在当前旋转的基础上进行一次旋转的迭代。这个差值得到降低。因此优化的过程可以如下面的公式表示：

   $$_E^{S}\hat q_{k+1}=_E^{S}\hat q_k-\mu \frac{\bigtriangledown f(_E^{S}\hat q_k,\ ^{E}\hat d,\ ^{S}\hat s)}{\left \Vert \bigtriangledown f(_E^{S}\hat q_k,\ ^{E}\hat d,\ ^{S}\hat s) \right \Vert},\ k=0,1,2...n $$

   显然，每一步迭代都会使得旋转四元数向$f$模长降低的方向逼近，即测量值和预测值的方向不断接近。梯度的具体推导如下：

   **...待续...**





   



   

