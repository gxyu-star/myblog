---
layout: post
title: "Estimate 6Dof Pose from IMU and Vision Matched Points"
author: "gxyu"
---

# <center>视觉2D-3D匹配点结合IMU 3DOF解Pose推导</center>


将IMU的3DOF数据利用起来，与视觉的2D-3D匹配点集耦合，进行6DOF的姿态解算。下面是理论推导过程：
$$\begin{equation} z_c \begin{bmatrix} x \\ y \\ 1 \end{bmatrix} = \begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} R | t \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix} \end{equation}$$

其中的旋转矩阵$R$表示Camera到模型的旋转，向量$t$表示模型在Camera坐标系下的位移。为了计算方便将模型的3D点先转到IMU的坐标系下，表示为$\begin{bmatrix} X_i & Y_i & Z_i & 1 \end{bmatrix}^T$。在当前的应用场景中，我们由VIO数据可以得到Camera相对于VIO世界坐标系的旋转$^{wv}R_c$，同时由手柄的3DOF可以得到IMU相对于手柄世界坐标系的旋转$^{wi}R_i$。其中VIO的世界坐标系和IMU的世界坐标系相差一个Yaw方向的旋转，记为$^{wv}R_{wi}$。IMU相对于其世界坐标系的旋转$^{wi}R_i$可以进一步分分解为三个轴的欧拉角的转动，从而得到三个旋转矩阵的分量，如下：
$$\begin{equation}^{wi}R_i = ^{wi}R_y * ^yR_p * ^pR_r \end{equation}$$

在IMU的3DOF解算中，由于重力的作用，可以得到相对比较准确的pitch和roll，而yaw的值往往伴随着漂移。因此，三个旋转分量中$^{wi}R_y$是不准确的。将公式$(1)$中的$\begin{bmatrix} R | t \end{bmatrix}$进行展开如下：

$$\begin{equation}\begin{bmatrix} R | t \end{bmatrix} = \begin{bmatrix} ^{wv}R_c^{-1} * ^{wv}R_{wi} *  ^{wi}R_y * ^yR_p * ^pR_r | t \end{bmatrix}\end{equation}$$

其中$^{wv}R_{wi} *  ^{wi}R_y$和$t$是未知数，其他都是已知的。因此Pose的解算就弱化为求yaw方向的旋转，以及三维平移。为了书写方便，将几个旋转矩阵和并：$^{wv}R_c^{-1} = R_1$，$^{wv}R_{wi} *  ^{wi}R_y = R_2$，$^yR_p * ^pR_r = R_3$，那么上式可以写成：

$$\begin{equation}\begin{bmatrix} R | t \end{bmatrix} = \begin{bmatrix} R_1 * R_2 * R_3 | t \end{bmatrix}\end{equation}$$

其中$R_1$和$R_3$是已知的，需要求解的是$R_2$和$t$，进一步将公式$(1)$写成如下形式：

$$\begin{equation}z_c \begin{bmatrix} x \\ y \\ 1 \end{bmatrix} = \begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} R_1 * R_2 * R_3 | t \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}\end{equation}$$

$$\Rightarrow z_c \begin{bmatrix} x \\ y \\ 1 \end{bmatrix} = \begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix}  R_1 * \begin{bmatrix} R_2 | R_1^{-1}t \end{bmatrix} \begin{bmatrix} R_3 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}$$

$$\Rightarrow \begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix}^{-1} z_c  \begin{bmatrix} x \\ y \\ 1 \end{bmatrix} =  \begin{bmatrix} R_1 * R_2 | t \end{bmatrix} \begin{bmatrix} R_3 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}$$

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix} =  \begin{bmatrix} R_1 * R_2 | t \end{bmatrix}  \begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}\end{equation}$$

其中
$$\begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix}=\begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix}^{-1} \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}$$

$$\begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}=\begin{bmatrix} R_3 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}$$

那么要求解的方程就变成了的公式$(6)$，为了更清楚的知道式中的未知变量，进一步将公式$(6)$展开。

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix} =  \begin{bmatrix} \begin{bmatrix} r_1 & r_2 & r_3 \\ r_4 & r_5 & r_6\\ r_7 & r_8 & r_9\end{bmatrix} \begin{bmatrix} c_a & -s_a & 0 \\ s_a & c_a & 0\\ 0 & 0 & 1\end{bmatrix} | \begin{bmatrix} t_0 \\ t_1 \\ t_2 \end{bmatrix} \end{bmatrix} \begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}\end{equation}$$

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix} =  \begin{bmatrix} \begin{bmatrix} r_1c_a+r_2s_a  & -r_1s_a+r_2c_a & r_3 \\ r_4c_a+r_5s_a  & -r_4s_a+r_5c_a & r_6 \\ r_7c_a+r_8s_a  & -r_7s_a+r_8c_a & r_9\end{bmatrix} | \begin{bmatrix} t_0 \\ t_1 \\ t_2 \end{bmatrix} \end{bmatrix} \begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}\end{equation}$$

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix} =  \begin{bmatrix} (r_1c_a+r_2s_a)X_i^{'}+ (-r_1s_a+r_2c_a)Y_i^{'} + r_3 Z_i^{'} + t_0 \\ (r_4c_a+r_5s_a)X_i^{'}+ (-r_4s_a+r_5c_a)Y_i^{'} + r_6 Z_i^{'} + t_1 \\ (r_7c_a+r_7s_a)X_i^{'}+ (-r_7s_a+r_8c_a)Y_i^{'} + r_9 Z_i^{'} + t_2 \end{bmatrix} \end{equation}$$

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ 1 \end{bmatrix} =  \begin{bmatrix} (r_1 X_i^{'} +r_2Y_i^{'})c_a + (r_2X_i^{'}-r_1Y_i^{'})s_a + r_3 Z_i^{'} + t_0 \\ (r_4 X_i^{'} +r_5Y_i^{'})c_a + (r_5X_i^{'}-r_4Y_i^{'})s_a + r_6 Z_i^{'} + t_1 \\ (r_7 X_i^{'} +r_8Y_i^{'})c_a + (r_8X_i^{'}-r_7Y_i^{'})s_a + r_9 Z_i^{'} + t_2 \end{bmatrix} \end{equation}$$

其中$c_a \ s_a \ t_0 \ t_1 \ t_2\ z_c$是未知的。接下来通过解线性方程解出这六个未知数的。将公式$(10)$进一步展开：

$$\begin{equation}\begin{cases} z_cx^{'}=(r_1 X_i^{'} +r_2Y_i^{'})c_a + (r_2X_i^{'}-r_1Y_i^{'})s_a + r_3 Z_i^{'} + t_0 \\z_cy^{'}=(r_4 X_i^{'} +r_5Y_i^{'})c_a + (r_5X_i^{'}-r_4Y_i^{'})s_a + r_6 Z_i^{'} + t_1\\ z_c = (r_7 X_i^{'} +r_8Y_i^{'})c_a + (r_8X_i^{'}-r_7Y_i^{'})s_a + r_9 Z_i^{'} + t_2\end{cases}\end{equation}$$

$$\begin{equation}\Rightarrow\begin{cases} x^{'}=\frac{(r_1 X_i^{'} +r_2Y_i^{'})c_a + (r_2X_i^{'}-r_1Y_i^{'})s_a + r_3 Z_i^{'} + t_0}{(r_7 X_i^{'} +r_8Y_i^{'})c_a + (r_8X_i^{'}-r_7Y_i^{'})s_a + r_9 Z_i^{'} + t_2} \\ y^{'}=\frac{(r_4 X_i^{'} +r_5Y_i^{'})c_a + (r_5X_i^{'}-r_4Y_i^{'})s_a + r_6 Z_i^{'} + t_1}{(r_7 X_i^{'} +r_8Y_i^{'})c_a + (r_8X_i^{'}-r_7Y_i^{'})s_a + r_9 Z_i^{'} + t_2}\end{cases}\end{equation}$$

$$\begin{equation}\Rightarrow\begin{cases} x^{'}((r_7 X_i^{'} +r_8Y_i^{'})c_a + (r_8X_i^{'}-r_7Y_i^{'})s_a + r_9 Z_i^{'} + t_2) =(r_1 X_i^{'} +r_2Y_i^{'})c_a + (r_2X_i^{'}-r_1Y_i^{'})s_a + r_3 Z_i^{'} + t_0 \\ y^{'} ((r_7 X_i^{'} +r_8Y_i^{'})c_a + (r_8X_i^{'}-r_7Y_i^{'})s_a + r_9 Z_i^{'} + t_2)=(r_4 X_i^{'} +r_5Y_i^{'})c_a + (r_5X_i^{'}-r_4Y_i^{'})s_a + r_6 Z_i^{'} + t_1\end{cases}\end{equation}$$

$$\begin{equation}\Rightarrow\begin{cases} [x^{'}(r_7 X_i^{'} +r_8Y_i^{'}) - (r_1 X_i^{'} +r_2Y_i^{'})]c_a +  [x^{'}(r_8X_i^{'}-r_7Y_i^{'}) - (r_2X_i^{'}-r_1Y_i^{'})]s_a  - t_0 +  x^{'}t_2 = r_3 Z_i^{'} - x^{'}r_9 Z_i^{'}\\ [y^{'} (r_7 X_i^{'} +r_8Y_i^{'}) - (r_4 X_i^{'} +r_5Y_i^{'})]c_a + [y^{'}(r_8X_i^{'}-r_7Y_i^{'}) - (r_5X_i^{'}-r_4Y_i^{'})]s_a  - t_1+ y^{'}t_2 = r_6 Z_i^{'} - y^{'}r_9 Z_i^{'}\end{cases}\end{equation}$$


将公式$(14)$表示成矩阵的形式：

$$\begin{equation}  A*B^T=C\end{equation}$$

其中：
 $$A=\begin{bmatrix} [x^{'}(r_7 X_i^{'} +r_8Y_i^{'}) - (r_1 X_i^{'} +r_2Y_i^{'})] & [x^{'}(r_8X_i^{'}-r_7Y_i^{'}) - (r_2X_i^{'}-r_1Y_i^{'})] & -1 & 0 & x^{'} \\ [y^{'} (r_7 X_i^{'} +r_8Y_i^{'}) - (r_4 X_i^{'} +r_5Y_i^{'})] & [y^{'}(r_8X_i^{'}-r_7Y_i^{'}) - (r_5X_i^{'}-r_4Y_i^{'})] & 0 & -1 & y^{'}\end{bmatrix} $$

$$B=\begin{bmatrix} c_a & s_a & t_0 & t_1 & t_2 \end{bmatrix}$$

$$c=\begin{bmatrix} r_3 Z_i^{'} - x^{'}r_9 Z_i^{'} \\ r_6 Z_i^{'} - y^{'}r_9 Z_i^{'} \end{bmatrix}$$

方程$(14)$是一个齐次线性方程组，至少需要三个点可以解出未知变量。即求解下列方程组：

$$\begin{equation}  \begin{bmatrix}[x_1^{'}(r_7 X_{i1}^{'} +r_8Y_{i1}^{'}) - (r_1 X_{i1}^{'} +r_2Y_{i1}^{'})] & [x_1^{'}(r_8X_{i1}^{'}-r_7Y_{i1}^{'}) - (r_2X_{i1}^{'}-r_1Y_{i1}^{'})] & -1 & 0 & x_1^{'} \\ [y_1^{'} (r_7 X_{i1}^{'} +r_8Y_{i1}^{'}) - (r_4 X_{i1}^{'} +r_5Y_{i1}^{'})] & [y_1^{'}(r_8X_{i1}^{'}-r_7Y_{i1}^{'}) - (r_5X_{i1}^{'}-r_4Y_{i1}^{'})] & 0 & -1 & y_1^{'} \\ [x_2^{'}(r_7 X_{i2}^{'} +r_8Y_{i2}^{'}) - (r_1 X_{i2}^{'} +r_2Y_{i2}^{'})] & [x_2^{'}(r_8X_{i2}^{'}-r_7Y_{i2}^{'}) - (r_2X_{i2}^{'}-r_1Y_{i2}^{'})] & -1 & 0 & x_2^{'} \\ [y_2^{'} (r_7 X_{i2}^{'} +r_8Y_{i2}^{'}) - (r_4 X_{i2}^{'} +r_5Y_{i2}^{'})] & [y_2^{'}(r_8X_{i2}^{'}-r_7Y_{i2}^{'}) - (r_5X_{i2}^{'}-r_4Y_{i2}^{'})] & 0 & -1 & y_2^{'} \\ [x_3^{'}(r_7 X_{i2}^{'} +r_8Y_{i2}^{'}) - (r_1 X_{i2}^{'} +r_2Y_{i2}^{'})] & [x_3^{'}(r_8X_{i3}^{'}-r_7Y_{i3}^{'}) - (r_2X_{i3}^{'}-r_1Y_{i3}^{'})] & -1 & 0 & x_3^{'} \\ [y_3^{'} (r_7 X_{i3}^{'} +r_8Y_{i3}^{'}) - (r_4 X_{i3}^{'} +r_5Y_{i3}^{'})] & [y_3^{'}(r_8X_{i3}^{'}-r_7Y_{i3}^{'}) - (r_5X_{i3}^{'}-r_4Y_{i3}^{'})] & 0 & -1 & y_3^{'}\end{bmatrix}*\begin{bmatrix} c_a \\s_a \\ t_0^{'} \\ t_1^{'} \\ t_2^{'}\end{bmatrix}=\begin{bmatrix} r_3 Z_{i1}^{'} - x_1^{'}r_9 Z_{i1}^{'} \\ r_6 Z_{i1}^{'} - y_1^{'}r_9 Z_{i1}^{'} \\ r_3 Z_{i2}^{'} - x_2^{'}r_9 Z_{i2}^{'} \\ r_6 Z_{i2}^{'} - y_2^{'}r_9 Z_{i2}^{'} \\ r_3 Z_{i3}^{'} - x_3^{'}r_9 Z_{i3}^{'} \\ r_6 Z_{i3}^{'} - y_3^{'}r_9 Z_{i3}^{'} \end{bmatrix} \end{equation}$$

其中$\begin{bmatrix} x_1^{'} & y_1^{'} & z_1^{'} \end{bmatrix}$和$\begin{bmatrix}X_{i1}^{'} & Y_{i1}^{'} & Z_{i1}^{'} \end{bmatrix}$，$\begin{bmatrix} x_2^{'} & y_2^{'} & z_2^{'} \end{bmatrix}$和$\begin{bmatrix}X_{i2}^{'} & Y_{i2}^{'} & Z_{i2}^{'} \end{bmatrix}$，$\begin{bmatrix} x_2^{'} & y_2^{'} & z_2^{'} \end{bmatrix}$和$\begin{bmatrix}X_{i3}^{'} & Y_{i3}^{'} & Z_{i3}^{'} \end{bmatrix}$是三组匹配的点，只要三组点不是共线的，以下矩阵就是满秩的$Rank(A) = 6$，那么非齐次线性方程组$(16)$就有唯一解。

至此，就解出了旋转和平移。