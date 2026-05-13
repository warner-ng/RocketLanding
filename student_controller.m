% MIMO I/O linearization on outputs (z, psi) with cascaded zero-dynamics
% management.  Three phases:
%   P1 RECOVERY -- when cos(theta+psi) is too small / negative
%   P2 DESCENT  -- normal cascade
%   P3 FINAL    -- below 3L, tighter z, softer y
%
% Returns optional 3rd output `diag` (struct of intermediate signals) that
% sim_rocket.m collects in its post-processing loop for plotting.
%
% Cascade structure (top to bottom = "outer to inner"):
%   y, ydot      -> theta_desired       [PD; theta as virtual control of y]
%   theta, thdot -> psi_cmd             [invert  ddtheta = -L*gam*sin(psi)*fT/J]
%   psi,   pdot  -> v_psi -> tau        [I/O lin: ddpsi = tau/JT]
%   z, zdot      -> v_z  -> fT          [I/O lin: ddz   = gam*cos(th+psi)*fT/m - g]
%
function [u, integrator_dx, diag] = student_controller(t, x_full, consts, ctrl)

    x   = x_full(1:9) ;
    y   = x(1) ;  z    = x(2) ;  th   = x(3) ;  psi  = x(4) ;
    dy  = x(5) ;  dz   = x(6) ;  dth  = x(7) ;  dpsi = x(8) ;
    m   = x(9) ;

    g    = consts.g ;
    gam  = consts.gamma ;
    L    = consts.L ;
    J    = consts.Jm * m ;        % matches sim_rocket.m line 68
    JT   = consts.JT ;

    % ---------------------------------------------------------------
    % Wrap theta into (-pi, pi] for control purposes (multiple-of-2pi
    % rotations are not penalized: see compute_score line 8).
    % ---------------------------------------------------------------
    th_w = atan2(sin(th), cos(th)) ;

    % cos(th + psi) drives the I/O law.  Use the wrapped angle (same value)
    cphi = cos(th_w + psi) ;
    sphi = sin(th_w + psi) ;

    % ---------------------------------------------------------------
    % Fuel-aware fT_max
    % ---------------------------------------------------------------
    fuel = m - ctrl.m_dry ;
    fT_max_eff = ctrl.fT_max ;
    if fuel <= 0
        fT_max_eff = ctrl.fT_min ;          % engine forced off by sim anyway
    elseif fuel < ctrl.fuel_taper
        fT_max_eff = ctrl.fT_max * max(0, fuel / ctrl.fuel_taper) ;
    end
    fT_hover = m * g / gam ;                % thrust to cancel gravity now

    % ---------------------------------------------------------------
    % PHASE: stateless, pure function of (cphi, z).  No persistent state,
    % no hysteresis -- those were incompatible with ode45 adaptive
    % stepping: when the integrator rejects a step and backtracks, the
    % persistent variable retained the "future-rejected" value and the
    % trajectory became tolerance-dependent.  Smooth blending below
    % replaces the discrete switches entirely.
    % ---------------------------------------------------------------
    if cphi < ctrl.cos_thresh_lo
        phase_state = 1 ;
    elseif z < ctrl.z_phase3
        phase_state = 3 ;
    else
        phase_state = 2 ;
    end
    % Continuous blend coefficient used by both theta_d and fT:
    %   blend = 0 in (former) P1 region
    %   blend = 1 in (former) P2/P3 region
    blend = max(0, min(1, (cphi - ctrl.cos_thresh_lo) / ...
                          (ctrl.cos_thresh_hi - ctrl.cos_thresh_lo))) ;

    % ---------------------------------------------------------------
    % Z-CHANNEL: design v_z (desired z-double-dot) using a
    % velocity-tracking law with altitude-dependent reference.
    %
    %   zdot_target(z) = -min(zdot_max, sqrt(2*a*(z-L)))
    %
    % Smooth: free-fall-like at altitude, decelerating to zdot_land at z=L.
    % ---------------------------------------------------------------
    z_err = max(0, z - L) ;                         % positive above pad; C^0 at L
    % Kinematic target: sqrt-shape lands softly (=0 at pad); no discrete jump.
    zdot_target = -min(ctrl.zdot_max, sqrt(2 * ctrl.a_brake * z_err)) ;

    % ---------------------------------------------------------------
    % Gain scheduling: smooth ramp from "mid" gains to "final" gains as
    % z descends from z_phase3 to 0.  Replaces the discrete Phase-3 switch.
    % ---------------------------------------------------------------
    g_fac = max(0, min(1, (ctrl.z_phase3 - z) / ctrl.z_phase3)) ;
    kz_p  = ctrl.kz_p * (1 + (ctrl.kz_scale_final - 1) * g_fac) ;
    kz_v  = ctrl.kz_v * (1 + (ctrl.kz_scale_final - 1) * g_fac) ;
    ky_p  = ctrl.ky_p * (1 + (ctrl.ky_scale_final - 1) * g_fac) ;
    ky_v  = ctrl.ky_v * (1 + (ctrl.ky_scale_final - 1) * g_fac) ;

    % v_z = desired z-double-dot
    v_z = -kz_p * z_err - kz_v * (dz - zdot_target) ;

    % ---------------------------------------------------------------
    % FT: smooth blend between recovery (low fT, safe when nearly horizontal)
    %     and the I/O-linearized formula (when cphi is large).  Uses the
    %     same `blend` coefficient as theta_d, so the entire controller is
    %     C^0 in cphi.  No discrete switching anywhere.
    % ---------------------------------------------------------------
    cphi_safe = max(cphi, ctrl.cos_min) ;     % guarantee positive denominator
    fT_io     = m * (v_z + g) / (gam * cphi_safe) ;
    fT_rec    = ctrl.fT_recovery_factor * fT_hover ;
    fT = blend * fT_io + (1 - blend) * fT_rec ;
    fT = min(max(fT, ctrl.fT_min), fT_max_eff) ;

    % ---------------------------------------------------------------
    % Y-CHANNEL: theta_desired as virtual control
    %   ddy = -gam*sin(th+psi)*fT/m + d/m   ~~  -g*(th+psi) for small angles
    %   hence theta_d ~ -(ky_p*y + ky_v*ydot)/g  but we simply use a PD
    %
    % FIX C: two-tier theta cap, linearly tapered between altitudes.
    %        Mid-flight up to pi/3 (60 deg) to give the y-channel enough
    %        horizontal-deceleration authority; pi/6 only at the landing
    %        window so the score constraint is met.
    % ---------------------------------------------------------------
    if z >= ctrl.z_theta_taper_hi
        theta_des_cap = ctrl.theta_des_max_mid ;
    elseif z <= ctrl.z_theta_taper_lo
        theta_des_cap = ctrl.theta_des_max_final ;
    else
        alpha = (z - ctrl.z_theta_taper_lo) / ...
                (ctrl.z_theta_taper_hi - ctrl.z_theta_taper_lo) ;
        theta_des_cap = ctrl.theta_des_max_final + ...
            alpha * (ctrl.theta_des_max_mid - ctrl.theta_des_max_final) ;
    end

    % theta_d construction (continuous version - replaces the discrete
    % "if Phase 1 then theta_d=0" override that caused psi_cmd to flip
    % +/- 0.35 at the phase boundary).
    %
    %   1) Compute y-PD raw output (with |y|<threshold velocity-only mode)
    %   2) Smoothly fade to 0 when cos(theta+psi) drops (was P1 territory)
    %      using a linear blend over [cos_thresh_lo, cos_thresh_hi].  This
    %      preserves the "attitude-priority reset" behavior that stabilized
    %      run 4, but without discrete jumps.
    %
    % SIGN: d(ddy)/dtheta = -gamma*fT/m < 0, so theta_d has the same sign
    %                       as (ky_p*y + ky_v*dy).
    % Full PD on y (position + velocity).  The smooth blend on cphi above
    % already suppresses the ringing that the y_pos_threshold switch was
    % supposed to break, so we no longer need a special-case for small |y|.
    theta_d_raw = +(ky_p * y + ky_v * dy) ;
    % `blend` was computed at the top with the phase decision.  Re-use it
    % so theta_d and fT share the same continuous fade.
    theta_d = blend * theta_d_raw ;
    theta_d = max(-theta_des_cap, min(theta_d, theta_des_cap)) ;

    % ---------------------------------------------------------------
    % THETA-CHANNEL: invert  ddtheta = -L*gam*sin(psi)*fT/J  to get psi_cmd
    %   v_theta = desired ddtheta  (PD on attitude error)
    %   sin(psi_cmd) = -v_theta * J / (L*gam*fT)
    % NOTE: uses ACTUAL fT (not nominal) so saturation propagates correctly
    % ---------------------------------------------------------------
    th_err  = th_w - theta_d ;
    v_theta = -ctrl.kth_p * th_err - ctrl.kth_v * dth ;

    % FIX B: drop the "fT > 1.0 else psi_cmd=0" gate.  fT_min = 0.1 is
    %        always valid; we only need to avoid literal division by ~0.
    %        Previously this gate silently disabled attitude control any
    %        time the z-channel saturated low.
    % fT_min = 0.1, so the denominator is never zero in practice.
    % Add a small floor anyway to be defensive.  No branch needed -> C^0.
    arg     = -v_theta * J / (L * gam * max(fT, ctrl.fT_min)) ;
    arg     = max(-ctrl.arcsin_clamp, min(arg, ctrl.arcsin_clamp)) ;
    psi_cmd = asin(arg) ;
    psi_cmd = max(-ctrl.psi_cmd_max, min(psi_cmd, ctrl.psi_cmd_max)) ;

    % ---------------------------------------------------------------
    % PSI-CHANNEL: I/O linearization (exact, ddpsi = tau/JT)
    % ---------------------------------------------------------------
    v_psi = -ctrl.kpsi_p * (psi - psi_cmd) - ctrl.kpsi_v * dpsi ;
    tau   = JT * v_psi ;
    tau   = max(-ctrl.tau_max, min(tau, ctrl.tau_max)) ;

    u = [fT ; tau] ;
    integrator_dx = [] ;

    % ---------------------------------------------------------------
    % Diagnostics (cheap; computed unconditionally - sim only collects
    % them when nargout >= 3 in the post-processing loop)
    % ---------------------------------------------------------------
    if nargout >= 3
        diag.phase       = phase_state ;
        diag.cphi        = cphi ;
        diag.fT          = fT ;
        diag.fT_max_eff  = fT_max_eff ;
        diag.fT_hover    = fT_hover ;
        diag.tau         = tau ;
        diag.theta_d     = theta_d ;
        diag.th_w        = th_w ;
        diag.psi_cmd     = psi_cmd ;
        diag.v_z         = v_z ;
        diag.zdot_target = zdot_target ;
        diag.fuel        = fuel ;
        diag.arg_arcsin  = NaN ;
        if fT > 1e-3
            diag.arg_arcsin = -v_theta * J / (L * gam * fT) ;
        end
    end
end
