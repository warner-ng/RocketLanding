# 算法简介

本文采用 **MIMO 输入输出线性化** 作为主框架，选取输出 $h(x)=[y,z]^\top$。首先由平动动力学得到
$$
\ddot y = -\frac{\gamma}{m} f_T \sin(\phi),\quad
\ddot z = \frac{\gamma}{m} f_T \cos(\phi) - g,
$$
其中 $\phi=\theta+\psi$。将其写成矩阵形式可得解耦矩阵
$$
\begin{bmatrix}\ddot y \\ \ddot z + g\end{bmatrix}=
\frac{\gamma}{m}
\begin{bmatrix}-\sin\phi \\ \cos\phi\end{bmatrix} f_T.
$$
因此我们引入虚拟加速度 $\mathbf{a}_T=[a_y,a_z+g]^\top$，并通过
$$
f_T = \frac{m}{\gamma}\sqrt{a_y^2+(a_z+g)^2},\quad
\phi_d = \operatorname{atan2}(-a_y,a_z+g)
$$
完成外环反解。由于该反解对姿态动态的零动态敏感，我们将姿态通道视为内环，并通过 backstepping 设计虚拟控制：令期望姿态为 $\theta_d=\phi_d$，得到
$$
\ddot\theta_d = -k_\theta(\theta-\theta_d) - k_{d\theta}\dot\theta,
$$
进而由几何关系反解得到期望舵角 $\psi_d$，再由力矩通道跟踪 $\psi_d$。当零动态难以直接闭环求解时，上述 backstepping 等价地将其“吸收”到姿态内环，保证外环反解的可实现性。

在此基础上，我们逐步引入补偿机制。首先定义 **cruise** 区域以稳定下降速度：当
$$
z>z_c,\ |\theta|<\theta_g,\ |\dot\theta|<\dot\theta_g,\ |y|<y_g
$$
时，将参考下降速度限定为 $\dot z_{\text{ref}}=-v_c$，避免姿态尚未稳定时过快下降。其次，在大角度区域引入 **large-angle 恢复**：构造 CLF
$$
s_\theta=\dot\theta+\lambda\theta,\quad
\ddot\theta=-\lambda\dot\theta-k s_\theta,
$$
并对 $\dot\theta$ 与 $\ddot\theta$ 进行限幅，以避免高角速导致的数值与执行器饱和。此时推力上限分段控制，并在高空/深度反转区使用更强的收敛增益与更小推力上限，提高回正可控性。

总体而言，该策略以输入输出线性化为骨架，通过 backstepping 保证姿态可实现，再结合 cruise 与 large-angle 的分段控制，使系统能够从大角度恢复并平滑进入稳定下降阶段。
