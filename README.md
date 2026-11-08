
<p>
	<a href="#rocket-landing-controller--full-design-document-en">English</a> |
	<a href="#火箭着陆控制器--详细设计文档-zh">中文</a>
</p>

# Rocket Landing Controller — Full Design Document (EN)

## 1. Task Overview

We design a controller for a planar rocket booster to land within 60 seconds. The system is a 9-state, 2-input, nonlinear control-affine model:

$$
x = [y, z, \theta, \psi, \dot y, \dot z, \dot\theta, \dot\psi, m]^\top,\quad
u = [f_T, \tau]^\top.
$$

The dynamics are given in matrix form by the project handout (Eq. (1)):

$$
\begin{bmatrix} m & & & & \\ & m & & & \\ & & J(m) & & \\ & & & J_T & \\ & & & & 1 \end{bmatrix}
\begin{bmatrix} \ddot y \\ \ddot z \\ \ddot\theta \\ \ddot\psi \\ \dot m \end{bmatrix}
+
\begin{bmatrix} 0 \\ mg \\ 0 \\ 0 \\ 0 \end{bmatrix}
=
\begin{bmatrix} -\gamma\sin(\psi+\theta) & 0 \\ \gamma\cos(\psi+\theta) & 0 \\ -L\gamma\sin\psi & 0 \\ 0 & 1 \\ -1 & 0 \end{bmatrix}
\begin{bmatrix} f_T \\ \tau \end{bmatrix}
+
\begin{bmatrix} d \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

The mass matrix is diagonal, so each row can be read on its own. Expanding each row, then dividing by the coefficient on the left:

| Row | Expanded | Divided by the coefficient | Physical meaning |
|---|---|---|---|
| 1 | $m\ddot y = -\gamma\sin(\theta+\psi)\,f_T + d$ | $\ddot y = -\dfrac{\gamma}{m}\sin(\theta+\psi)\,f_T + \dfrac{d}{m}$ | Horizontal Newton's law: horizontal component of the thrust $\gamma f_T$ plus the wind disturbance $d$ |
| 2 | $m\ddot z + mg = \gamma\cos(\theta+\psi)\,f_T$ | $\ddot z = \dfrac{\gamma}{m}\cos(\theta+\psi)\,f_T - g$ | Vertical Newton's law: vertical component of the thrust against gravity |
| 3 | $J(m)\,\ddot\theta = -L\gamma\sin(\psi)\,f_T$ | $\ddot\theta = -\dfrac{L\gamma}{J(m)}\sin(\psi)\,f_T$ | Body rotation: the gimbal angle $\psi$ tilts the thrust off the COM, giving a torque with lever arm $L$ |
| 4 | $J_T\,\ddot\psi = \tau$ | $\ddot\psi = \dfrac{\tau}{J_T}$ | Gimbal: the torque $\tau$ directly drives the nozzle, with nozzle inertia $J_T$ |
| 5 | $\dot m = -f_T$ | $\dot m = -f_T$ | Mass: fuel is burned at rate $f_T$ ($\gamma$ is the exhaust velocity, so the thrust is $\gamma f_T$) |

Key structural fact: $\tau$ only affects $\ddot\psi$, while $f_T$ affects $\ddot y$, $\ddot z$, $\ddot\theta$, and $\dot m$ simultaneously. This coupling drives the architecture.

## 2. Approach 1: Backstepping (Rejected)

The system does not satisfy strict feedback form because $f_T$ appears in multiple channels (translation and rotation) at once. A direct backstepping cascade is therefore invalid. We instead use input-output linearization on appropriate outputs.

## 3. Approach 2: I/O Linearization on $(y,z)$ (Rejected)

Differentiating outputs twice yields the decoupling matrix
$$
A(x) = \begin{bmatrix}
-\gamma\sin(\theta+\psi)/m & 0 \\
\gamma\cos(\theta+\psi)/m & 0
\end{bmatrix},
$$
which is rank-deficient because $\tau$ does not appear in $\ddot y$ or $\ddot z$. In other words, the vector relative degree does not exist for this output choice, so the linearizing law $u = A(x)^{-1}(v - b(x))$ cannot be solved, and MIMO I/O linearization is impossible with $(y,z)$.

## 4. Final Architecture: I/O Linearization on $(z,\psi)$ with Zero-Dynamics Management

Choose outputs $h(x)=[z,\psi]^\top$. The decoupling matrix becomes diagonal:
$$
A(x) = \begin{bmatrix}
\gamma\cos(\theta+\psi)/m & 0 \\
0 & 1/J_T
\end{bmatrix},
$$
which is full rank when $\cos(\theta+\psi)\neq 0$. The I/O linearizing control law is
$$
f_T = \frac{m\,(v_1+g)}{\gamma\cos(\theta+\psi)},\quad
\tau = J_T v_2,
$$
yielding the linearized channels $\ddot z = v_1$, $\ddot\psi = v_2$. We implement PD laws for $v_1$ and $v_2$, while the remaining dynamics $(\theta,\dot\theta,y,\dot y)$ are stabilized through a backstepping-style virtual control chain.

### 4.1 Zero Dynamics and Backstepping Chain

1) **Horizontal control**: treat $\theta$ as a virtual control for $y$,
$$
\theta_d = -k_y y - k_{dy} \dot y,
$$
with saturation to avoid excessive tilt.

2) **Attitude control**: define a desired angular acceleration
$$
\ddot\theta_d = -k_\theta (\theta-\theta_d) - k_{d\theta}\dot\theta,
$$
and invert the attitude dynamics to obtain a gimbal command $\psi_d$.

3) **Gimbal control**: set $v_2$ to track $\psi_d$ through a PD law on $(\psi,\dot\psi)$.

This chain resolves the zero dynamics without requiring explicit closed-form stabilization of the internal subsystem.

## 5. Phase Logic

### Phase A — Large-Angle Recovery
When $|\theta|$ is large (or $|\dot\theta|$ exceeds a gate), we stop prioritizing lateral tracking and focus on attitude recovery. Concretely, we set the thrust direction target to upright, $\phi_d=0$, then compute a stabilizing angular acceleration via a CLF. First define
$$
s_\theta = \dot\theta + \lambda\theta,\quad
\ddot\theta = -\lambda\dot\theta - k s_\theta,
$$
then apply rate and acceleration limits:
$$
\dot\theta_{\text{eff}} = \operatorname{sat}(\dot\theta,\,-\dot\theta_{\max},\,\dot\theta_{\max}),\quad
\ddot\theta_{\text{cmd}} = \operatorname{sat}(\ddot\theta,\,-\ddot\theta_{\max},\,\ddot\theta_{\max}).
$$
Using the attitude channel $\ddot\theta = -\tfrac{L\gamma}{J(m)}\sin(\psi) f_T$, we realize the commanded acceleration with bounded gimbal and thrust: $|\psi|\le\psi_{\max}$ and $f_T\le f_{T,\max}$. In high-altitude or inverted regimes, these bounds and gains are scheduled more conservatively. This yields a stable recovery phase that restores attitude before resuming normal lateral control.

### Phase B — Cruise Descent
When the attitude is stable and altitude is high, we enforce a constant descent speed $\dot z_{\text{ref}}=-v_c$ to avoid aggressive descent while the attitude settles. Cruise entry requires
$$
z>z_c,\ |\theta|<\theta_g,\ |\dot\theta|<\dot\theta_g,\ |y|<y_g.
$$

### Phase C — Terminal Approach
Near the ground, descent speed is reduced via blended caps to satisfy touchdown constraints.


---

# 火箭着陆控制器 — 详细设计文档 (ZH)

## 1. 任务概述

目标是在 60 秒内将二维火箭回收到着陆点附近。系统为 9 状态、2 输入的非线性系统：
$$
x = [y, z, \theta, \psi, \dot y, \dot z, \dot\theta, \dot\psi, m]^\top,\quad
u = [f_T, \tau]^\top.
$$
题目给出的动力学矩阵形式（项目说明式 (1)）：

$$
\begin{bmatrix} m & & & & \\ & m & & & \\ & & J(m) & & \\ & & & J_T & \\ & & & & 1 \end{bmatrix}
\begin{bmatrix} \ddot y \\ \ddot z \\ \ddot\theta \\ \ddot\psi \\ \dot m \end{bmatrix}
+
\begin{bmatrix} 0 \\ mg \\ 0 \\ 0 \\ 0 \end{bmatrix}
=
\begin{bmatrix} -\gamma\sin(\psi+\theta) & 0 \\ \gamma\cos(\psi+\theta) & 0 \\ -L\gamma\sin\psi & 0 \\ 0 & 1 \\ -1 & 0 \end{bmatrix}
\begin{bmatrix} f_T \\ \tau \end{bmatrix}
+
\begin{bmatrix} d \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

质量矩阵是对角的，每一行可以单独看。逐行展开，再除以左边的系数：

| 行 | 展开 | 除以系数 | 物理含义 |
|---|---|---|---|
| 1 | $m\ddot y = -\gamma\sin(\theta+\psi)\,f_T + d$ | $\ddot y = -\dfrac{\gamma}{m}\sin(\theta+\psi)\,f_T + \dfrac{d}{m}$ | 水平方向牛顿第二定律：推力 $\gamma f_T$ 的水平分量加上风扰动 $d$ |
| 2 | $m\ddot z + mg = \gamma\cos(\theta+\psi)\,f_T$ | $\ddot z = \dfrac{\gamma}{m}\cos(\theta+\psi)\,f_T - g$ | 竖直方向牛顿第二定律：推力的竖直分量对抗重力 |
| 3 | $J(m)\,\ddot\theta = -L\gamma\sin(\psi)\,f_T$ | $\ddot\theta = -\dfrac{L\gamma}{J(m)}\sin(\psi)\,f_T$ | 箭体转动：喷管偏角 $\psi$ 让推力偏离质心，以力臂 $L$ 产生力矩 |
| 4 | $J_T\,\ddot\psi = \tau$ | $\ddot\psi = \dfrac{\tau}{J_T}$ | 喷管摆动：力矩 $\tau$ 直接驱动喷管，$J_T$ 是喷管的转动惯量 |
| 5 | $\dot m = -f_T$ | $\dot m = -f_T$ | 质量变化：燃料以 $f_T$ 的速率消耗（$\gamma$ 是燃气喷出速度，所以推力为 $\gamma f_T$） |

核心结构特征：$\tau$ 仅作用于 $\ddot\psi$，而 $f_T$ 同时影响平动、转动与质量变化。

## 2. Backstepping 方案（不可直接用）

由于 $f_T$ 同时进入多个通道，系统不满足严格反馈结构，常规 backstepping 不成立。

## 3. 以 $(y,z)$ 为输出的 I/O 线性化（不可行）

两个输出各求导两次后，$\tau$ 都没有出现，解耦矩阵第二列为 0、秩为 1。也就是说这组输出的向量相对阶不存在，解不出线性化控制律 $u = A(x)^{-1}(v - b(x))$，因此弃用。

## 4. 最终方案：以 $(z,\psi)$ 为输出的 I/O 线性化 + 零动态管理

选择输出 $h(x)=[z,\psi]^\top$，解耦矩阵对角且在 $\cos(\theta+\psi)\neq 0$ 时满秩，得到
$$
f_T = \frac{m\,(v_1+g)}{\gamma\cos(\theta+\psi)},\quad
\tau = J_T v_2,
$$
实现 $\ddot z=v_1$ 与 $\ddot\psi=v_2$。其余状态通过 backstepping 风格的虚拟控制链稳定。

### 4.1 零动态与虚拟控制链

1) 以 $\theta$ 作为 $y$ 的虚拟控制：
$$
\theta_d = -k_y y - k_{dy} \dot y,
$$
2) 姿态误差转化为期望角加速度：
$$
\ddot\theta_d = -k_\theta (\theta-\theta_d) - k_{d\theta}\dot\theta.
$$
3) 通过姿态动力学反解得到 $\psi_d$，再以 $v_2$ 跟踪 $\psi_d$。

## 5. 分段逻辑

**大角度恢复**：设定 $\phi_d=0$ 强制竖直回正，构造 $s_\theta=\dot\theta+\lambda\theta$，并计算
$$
\ddot\theta=-\lambda\dot\theta-k s_\theta,
$$
随后对 $\dot\theta$ 与 $\ddot\theta$ 进行限幅，得到 $\dot\theta_{\text{eff}}$ 与 $\ddot\theta_{\text{cmd}}$。结合
$$
\ddot\theta=-\frac{L\gamma}{J(m)}\sin(\psi) f_T,
$$
在给定 $\psi_{\max}$ 与 $f_{T,\max}$ 的约束下计算可行的控制，并在高空/反转区进行增益与推力上限分段调度。  
**Cruise**：满足 $z>z_c,|\theta|<\theta_g,|\dot\theta|<\dot\theta_g,|y|<y_g$ 时采用恒定下降速度。  
**终端接近**：近地面降低下降速度，保证落地约束。

