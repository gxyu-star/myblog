---
layout: post
title: "Estimate 6Dof Pose from IMU and Vision Matched Points"
author: "gxyu"
---

# <center>视觉2D-3D匹配点结合IMU 3DOF解Pose推导</center>
<center>于国星</center>
<center>2021-11-19</center>

## 

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

$$\Rightarrow R_1^{-1} * \begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix}^{-1} z_c  \begin{bmatrix} x \\ y \\ 1 \end{bmatrix} =  \begin{bmatrix} R_2 | R_1^{-1}t \end{bmatrix} \begin{bmatrix} R_3 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}$$

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ z^{'} \end{bmatrix} =  \begin{bmatrix} R_2 | t^{'} \end{bmatrix}  \begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}\end{equation}$$

其中
$$\begin{bmatrix} x^{'} \\ y^{'} \\ z^{'} \end{bmatrix}=R_1^{-1} * \begin{bmatrix} f_x & 0 &c_x \\ 0 & f_y & c_y\\ 0 & 0 & 1 \end{bmatrix}^{-1} \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}$$

$$\begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}=\begin{bmatrix} R_3 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} X_i \\ Y_i \\ Z_i \\ 1 \end{bmatrix}$$

$$t^{'} = R_1^{-1}t$$

那么要求解的方程就变成了的公式$(6)$，为了更清楚的知道式中的未知变量，进一步将公式$(6)$展开。

$$\begin{equation}\Rightarrow z_c  \begin{bmatrix} x^{'} \\ y^{'} \\ z^{'} \end{bmatrix} =  \begin{bmatrix} c_a & -s_a & 0 & t_0^{'} \\ s_a & c_a & 0 & t_1^{'}\\ 0 & 0 & 1 &t_2^{'}\end{bmatrix}  \begin{bmatrix} X_i^{'} \\ Y_i^{'} \\ Z_i^{'} \\ 1 \end{bmatrix}\end{equation}$$

其中$c_a \ s_a \ t_0 \ t_1 \ t_2\ z_c$是未知的。接下来通过解线性方程解出这六个未知数的。将公式$(7)$进一步展开：

$$\begin{equation}\begin{cases} z_cx^{'}=c_aX_i^{'} -s_aY_i^{'}+t_0^{'} \\z_cy^{'}=s_aX_i^{'} +c_aY_i^{'}+t_1^{'}\\ z_cz^{'}=Z_i^{'}+t_2^{'}\end{cases}\end{equation}$$

$$\begin{equation}\Rightarrow\begin{cases} c_aX_i^{'} -s_aY_i^{'}+t_0^{'}-z_cx^{'}=0 \\s_aX_i^{'} +c_aY_i^{'}+t_1^{'}-z_cy^{'}=0\\ Z_i^{'}+t_2^{'}-z_cz^{'} =0\end{cases}\end{equation}$$

将公式$(9)$表示成矩阵的形式：

$$\begin{equation}  A*B^T=C^T \end{equation}$$

其中：
 $$A=\begin{bmatrix} X_i^{'} & -Y_i^{'} & 1 & 0 & 0 & -x^{'} \\ Y_i^{'} & X_i^{'} & 0 & 1 & 0 & -y^{'} \\ 0 & 0 & 0 & 0 & 1 & -z^{'}\end{bmatrix} $$

$$B=\begin{bmatrix} c_a & s_a & t_0^{'} & t_1^{'} & t_2^{'} & z_c \end{bmatrix}$$

$$C = \begin{bmatrix} 0 & 0 & -Z_i^{'} \end{bmatrix}$$

方程$(10)$是一个非齐次线性方程组，其中的$A$和$C$都是已知的，向量$B$是待求解的。六个未知数需要六个线性不相关的方程，一组2D-3D匹配点对，可以列出三个方程，那么两组2D-3D匹配点对就可以求解所有未知数。即求解下列方程组：

$$\begin{equation}  \begin{bmatrix} X_{1i}^{'} & -Y_{1i}^{'} & 1 & 0 & 0 & -x_1^{'} \\ Y_{1i}^{'} & X_{1i}^{'} & 0 & 1 & 0 & -y_1^{'} \\ 0 & 0 & 0 & 0 & 1 & -z_1^{'} \\ X_{2i}^{'} & -Y_{2i}^{'} & 1 & 0 & 0 & -x_2^{'} \\ Y_{2i}^{'} & X_{2i}^{'} & 0 & 1 & 0 & -y_2^{'} \\ 0 & 0 & 0 & 0 & 1 & -z_2^{'}\end{bmatrix}*\begin{bmatrix} c_a \\s_a \\ t_0^{'} \\ t_1^{'} \\ t_2^{'} \\ z_c \end{bmatrix}=\begin{bmatrix} 0 \\ 0 \\ -Z_{1i}^{'}\\0 \\ 0 \\ -Z_{2i}^{'} \end{bmatrix} \end{equation}$$

其中$\begin{bmatrix} x_1^{'} & y_1^{'} & z_1^{'} \end{bmatrix}$和$\begin{bmatrix}X_{1i}^{'} & Y_{1i}^{'} & Z_{1i}^{'} \end{bmatrix}$，$\begin{bmatrix} x_2^{'} & y_2^{'} & z_2^{'} \end{bmatrix}$和$\begin{bmatrix}X_{2i}^{'} & Y_{2i}^{'} & Z_{2i}^{'} \end{bmatrix}$是两组匹配的点，只要两组点不是重叠的，以下矩阵就是满秩的$Rank(A) = 6$，那么非齐次线性方程组$(11)$就有唯一解。

$$A=\begin{bmatrix} X_{1i}^{'} & -Y_{1i}^{'} & 1 & 0 & 0 & -x_1^{'} \\ Y_{1i}^{'} & X_{1i}^{'} & 0 & 1 & 0 & -y_1^{'} \\ 0 & 0 & 0 & 0 & 1 & -z_1^{'} \\ X_{2i}^{'} & -Y_{2i}^{'} & 1 & 0 & 0 & -x_2^{'} \\ Y_{2i}^{'} & X_{2i}^{'} & 0 & 1 & 0 & -y_2^{'} \\ 0 & 0 & 0 & 0 & 1 & -z_2^{'}\end{bmatrix}$$

$$\begin{equation}B^T=\begin{bmatrix} c_a \\s_a \\ t_0^{'} \\ t_1^{'} \\ t_2^{'} \\ z_c \end{bmatrix}=\begin{bmatrix} X_{1i}^{'} & -Y_{1i}^{'} & 1 & 0 & 0 & -x_1^{'} \\ Y_{1i}^{'} & X_{1i}^{'} & 0 & 1 & 0 & -y_1^{'} \\ 0 & 0 & 0 & 0 & 1 & -z_1^{'} \\ X_{2i}^{'} & -Y_{2i}^{'} & 1 & 0 & 0 & -x_2^{'} \\ Y_{2i}^{'} & X_{2i}^{'} & 0 & 1 & 0 & -y_2^{'} \\ 0 & 0 & 0 & 0 & 1 & -z_2^{'}\end{bmatrix}^{-1} * \begin{bmatrix} 0 \\ 0 \\ -Z_{1i}^{'}\\0 \\ 0 \\ -Z_{2i}^{'} \end{bmatrix}\end{equation}$$

至此，就解出了旋转和平移，以及尺度。由$t^{'} = R_1^{-1}t$可以得到平移：
$$t = R_1t^{'}$$