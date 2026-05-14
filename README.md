
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

The dynamics (after inverting the diagonal mass matrix) are:
$$
\ddot y = -\frac{\gamma}{m} \sin(\theta+\psi) f_T + \frac{d}{m},\quad
\ddot z = \frac{\gamma}{m} \cos(\theta+\psi) f_T - g,
$$
$$
\ddot\theta = -\frac{L\gamma}{J(m)} \sin(\psi) f_T,\quad
\ddot\psi = \frac{\tau}{J_T},\quad
\dot m = -f_T.
$$

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
which is rank-deficient because $\tau$ does not appear in $\ddot y$ or $\ddot z$. Hence MIMO I/O linearization is impossible with $(y,z)$. Moreover, the resulting zero dynamics are non-minimum phase (unstable attitude).

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
动力学为：
$$
\ddot y = -\frac{\gamma}{m} \sin(\theta+\psi) f_T + \frac{d}{m},\quad
\ddot z = \frac{\gamma}{m} \cos(\theta+\psi) f_T - g,
$$
$$
\ddot\theta = -\frac{L\gamma}{J(m)} \sin(\psi) f_T,\quad
\ddot\psi = \frac{\tau}{J_T},\quad
\dot m = -f_T.
$$
核心结构特征：$\tau$ 仅作用于 $\ddot\psi$，而 $f_T$ 同时影响平动、转动与质量变化。

## 2. Backstepping 方案（不可直接用）

由于 $f_T$ 同时进入多个通道，系统不满足严格反馈结构，常规 backstepping 不成立。

## 3. 以 $(y,z)$ 为输出的 I/O 线性化（不可行）

对应解耦矩阵秩亏，且零动态不稳定，因此弃用。

## 4. 最终方案：以 $(z,\psi)$ 为输出的 I/O 线性化 + 零动态管理

选择输出 $h(x)=[z,\psi]^\top$，解耦矩阵对角且在 $\cos(\theta+\psi)\neq 0$ 时满秩，得到
$$
f_T = \frac{m\,(v_1+g)}{\gamma\cos(\theta+\psi)},\quad
	au = J_T v_2,
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

