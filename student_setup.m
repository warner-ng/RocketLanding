% Function to setup and perform any one-time computations
% Input parameters
%   x0     - initial state x=[y,z,th,psi,dy,dz,dth,dpsi,m]^T
%   consts - structure with system constants
% Output parameters
%   ctrl         - student-defined control parameters
%   student_data - id + nick_name for the leaderboard
%
% --------------------------------------------------------------------
% DESIGN: MIMO I/O linearization on outputs (z, psi).
%   Decoupling matrix is diagonal:  diag( gamma cos(th+psi)/m , 1/JT )
%   Full rank whenever cos(th+psi) > 0 (rocket pointing "up-ish").
%   Zero dynamics: (theta, thetadot, y, ydot) handled by cascading
%      y, ydot      ->  theta_desired   (PD)
%      theta, thdot ->  psi_cmd         (invert the theta-dotdot eqn)
%
% PHASES:
%   P1 RECOVERY: cos(th+psi) < cos_thresh_lo  (rocket too tilted/inverted)
%      - Use moderate fT (cannot use I/O law: it would demand fT<0 or huge fT)
%      - Actively command psi to torque theta toward 0 via the same inversion
%        formula but with theta_desired = 0
%   P2 DESCENT: cos(th+psi) > cos_thresh_hi  (hysteresis)
%      - Full cascade: y -> theta_d -> psi_cmd -> tau ; z -> fT
%   P3 FINAL:   z < 3*L
%      - Tighten z gains, soften y gains (no wind below 2L anyway)
%
% FUEL: dot(m) = -fT  =>  hover burn rate = m*g/gamma ~ 1.7 kg/s.
%   60 s of pure hover ~ 100 kg.  Budget ~50 kg for maneuvers.
% --------------------------------------------------------------------
function [ctrl, student_data] = student_setup(x0, consts)

    student_data.id        = 3042103210 ;
    student_data.nick_name = 'Warner Wu' ;

    g     = consts.g ;
    gam   = consts.gamma ;
    m0    = x0(9) ;

    % ---- z-channel: PURE velocity tracking (kz_p=0, FIX A) ----
    %    zdot_target(z) = -min(zdot_max, sqrt(2 a_brake (z-L)))
    %    encodes the descent profile; no extra position term needed.
    %    A nonzero kz_p at altitude demands negative thrust => fT clamps
    %    to fT_min, the cascade dies.
    ctrl.kz_p = 0.00 ;
    ctrl.kz_v = 1.20 ;
    ctrl.zdot_max  = 8.0 ;
    ctrl.a_brake   = 0.5 ;        % gentler near-pad deceleration -> more time
                                  % at low altitude for cascade to settle
    ctrl.zdot_land = 1.0 ;

    % ---- y-channel (horizontal) - uses theta as virtual input ----
    %   Closed loop (small angle):  ddy = -g*theta_d = -g*(ky_p*y + ky_v*dy)
    %   Char poly:  s^2 + g*ky_v*s + g*ky_p = 0
    %   Critical damping:  ky_v = 2*sqrt(ky_p/g)  ==> for ky_p=0.05, ky_v~0.143
    ctrl.ky_p = 0.05 ;
    ctrl.ky_v = 0.15 ;            % near-critical damping

    % Structural switch: bandwidth of position-PD on y is omega_y ~ sqrt(g/|y|),
    % which DIVERGES as |y| -> 0, eventually overtaking the theta cascade
    % (~1 rad/s).  Inside |y| < y_pos_threshold drop the position term and
    % only damp velocity, breaking the limit-cycle near touchdown.
    ctrl.y_pos_threshold = 15 ;   % m, well inside the +/-20 m landing pad
    % FIX C: theta_des cap is two-tier (mid-flight vs landing window)
    ctrl.theta_des_max_mid   = pi/3 ;     % 60 deg authority in cruise
    ctrl.theta_des_max_final = pi/6 ;     % 30 deg at touchdown (score limit)
    ctrl.z_theta_taper_hi    = 60 ;       % above this z: mid cap
    ctrl.z_theta_taper_lo    = 30 ;       % below this z: final cap

    % ---- theta-channel (attitude) gains - generates psi_cmd via arcsin ----
    ctrl.kth_p = 3.0 ;
    ctrl.kth_v = 3.5 ;
    ctrl.psi_cmd_max  = 0.35 ;    % rad, hard cap on commanded gimbal angle
    ctrl.arcsin_clamp = 0.90 ;    % keep arcsin argument away from +/-1

    % ---- psi-channel (gimbal) gains - I/O linearized, double integrator ----
    % Push omega_psi up: BW now ~5.5 rad/s vs theta BW ~1.7 rad/s -> 3.2x
    % separation (vs the 2.2x we had).  tau saturates more but that's the
    % correct place to spend authority.
    ctrl.kpsi_p = 30.0 ;
    ctrl.kpsi_v = 10.0 ;

    % ---- Phase logic (FIX: trigger sooner so theta=57deg starts recovery) ----
    ctrl.cos_thresh_lo = 0.50 ;   % enter P1 when cos < lo (was 0.30)
    ctrl.cos_thresh_hi = 0.70 ;   % leave P1 when cos > hi (hysteresis)
    ctrl.z_phase3      = 3*consts.L ;   % enter final phase below 3L = 30 m

    % Recovery thrust (just enough to slow descent + give torque authority)
    %   dot(m) = -fT, so fT_rec = 1.5*m*g/gamma uses ~2.5 kg/s
    ctrl.fT_recovery_factor = 2.0 ;     % times hover thrust at current m
                                        % (1.5 lost too much altitude during recovery)

    % Phase-3 gain multipliers
    ctrl.kz_scale_final = 1.5 ;
    ctrl.ky_scale_final = 0.5 ;

    % ---- Fuel safeguarding ----
    ctrl.m_dry      = consts.m_nofuel ;
    ctrl.fuel_taper = 10.0 ;      % start tapering fT_max when fuel < this (kg)

    % ---- Numerical guards ----
    ctrl.cos_min   = 0.05 ;       % never divide by less than this in I/O law
    ctrl.fT_min    = consts.min.fT ;
    ctrl.fT_max    = consts.max.fT ;
    ctrl.tau_max   = consts.max.tau ;

    % ---- Initial-condition memo for trajectory shaping ----
    ctrl.z0   = x0(2) ;
    ctrl.t0   = 0 ;

    % ---- Simulator hooks ----
    ctrl.ode_type      = 0 ;      % ode45
    ctrl.N_integrators = 0 ;

    % ---- Diagnostic flag (read by post-processing in sim_rocket) ----
    ctrl.want_diag = true ;
end
