# Main-Algorithm Convergence and ROA (Theory)

This note provides a **theoretical** (non-experimental) local convergence rate and a conservative region-of-attraction (ROA) characterization for the **main algorithm only** (I/O linearization + backstepping chain). Cruise/large-angle add-ons are excluded.

## 1) Local Exponential Rates (Linearized)

We linearize around small angles and the nominal descent operating point. The closed-loop channels are second-order and their slowest poles determine the global rate.

### Vertical channel (z)
We use
$$
\dot z_{\text{ref}}\approx -\frac{v_{z\max}}{z_{\text{slow}}}(z-L),\quad
\ddot e_z + k_{zv}\dot e_z + k_{zv}\frac{v_{z\max}}{z_{\text{slow}}}e_z = 0.
$$
With $k_{zv}=0.6$, $v_{z\max}=30$, $z_{\text{slow}}=100$:
$$
\ddot e_z + 0.6\dot e_z + 0.18 e_z=0,
$$
so the poles are $-0.3\pm 0.3j$ and the **rate** is $\alpha_z\approx 0.3\,\text{s}^{-1}$.

### Attitude channel (theta)
The inner-loop is
$$
\ddot\theta + k_{d\theta}\dot\theta + k_\theta\theta=0.
$$
With $k_{d\theta}=2.5$, $k_\theta=1.2$, poles are approximately $-0.648$ and $-1.852$. Thus
$$
\alpha_\theta\approx 0.648\,\text{s}^{-1}.
$$

### Gimbal channel (psi)
$$
\ddot\psi + k_{d\psi}\dot\psi + k_\psi\psi=0.
$$
With $k_{d\psi}=18$, $k_\psi=80$, poles are $-8$ and $-10$, so
$$
\alpha_\psi\approx 8\,\text{s}^{-1}.
$$

### Overall main-algorithm rate
The backstepping chain is a cascade, so a conservative **overall exponential rate** is
$$
\alpha \approx \min\{\alpha_z,\alpha_\theta,\alpha_\psi\}=0.3\,\text{s}^{-1}.
$$
This is the **dominant** rate for small deviations in the main control regime.

## 2) Conservative ROA Conditions (Non-experimental)

The ROA is limited by I/O linearization feasibility and actuator bounds. A conservative local ROA is defined by the following **necessary** conditions:

1) **Decoupling matrix non-singularity**
$$
\cos(\theta+\psi)\ge c_0>0.
$$
With a conservative margin $c_0=0.3$,
$$
|\theta+\psi|\le \arccos(0.3)\approx 1.266\,\text{rad}.
$$

2) **Gimbal bound**
$$
|\psi|\le \psi_{\max}=25^\circ\approx 0.436\,\text{rad}.
$$

3) **Thrust feasibility**
From
$$
f_T=\frac{m(v_1+g)}{\gamma\cos(\theta+\psi)},
$$
require
$$
0 < f_T\le f_{T,\max},
$$
which yields a bound on $v_1$ given $m$ and $\theta+\psi$.

4) **Backstepping consistency**
The attitude inversion uses a bounded $\psi_d$ (limited by $\psi_{\max}$). Thus the implied $\ddot\theta_d$ must satisfy
$$
\left|\frac{J(m)\,\ddot\theta_d}{L\gamma f_T}\right|\le \sin(\psi_{\max}).
$$

Under these constraints, the linearized Lyapunov function $V=x^TPx$ yields
$$
\dot V\le -\lambda_{\min}(Q)\|x\|^2+2\|P\|L\|x\|^3,
$$
so a (conservative) local ROA radius is
$$
\|x\|<\frac{\lambda_{\min}(Q)}{2\|P\|L},
$$
where $L$ bounds the quadratic nonlinearity remainder (e.g., $|\sin x - x|\le x^2/2$).

## 3) Takeaway

- The main algorithm is **locally exponentially stable** with a conservative rate of about **$0.3\,\text{s}^{-1}$**, dominated by the $z$-channel.
- The ROA is bounded by **(i)** $|\theta+\psi|$ away from $\pi/2$, **(ii)** gimbal and thrust limits, and **(iii)** backstepping inversion feasibility.
- This is a theoretical bound, independent of simulation-based domain estimation.
