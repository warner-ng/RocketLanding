% Student supplied function to compute the control input at each instant of time
% Input parameters:
%   t  -  Time (in seconds) since start of simulation
%   x - state of the rocket, x=[y,z,th,psi,dy,dz,dth,dpsi,m, integrator_states]^T
%   consts - structure that contains various system constants
%   ctrl  -  any student defined control parameters
% Output parameters:
%   u  -  [thrust; torque] - two inputs to the rocket
% integrator_dx - number of integroators.  You will need to provide derivative 
% of the integrators and ode45 will integrate this.

function [u, integrator_dx] = student_controller(t, x_full, consts, ctrl)
    if(size(x_full, 2) > 1)
        x_full = x_full(:, 1) ;
    end

    % Extract state
    x = x_full(1:9) ; % The first 9 states corresponding to the rocket states

    y = x(1) ;
    z = x(2) ;
    th = wrap_angle(x(3)) ;
    psi = wrap_angle(x(4)) ;
    dy = x(5) ;
    dz = x(6) ;
    dth = x(7) ;
    dpsi = x(8) ;
    m = x(9) ;

    % ---- Section 51 MIMO feedback linearization, outer loop ----
    % yddot = -(gamma/m) fT sin(phi), zdotdot = (gamma/m) fT cos(phi) - g,
    % where phi = theta + psi.  Therefore [ay; az + g] is the virtual
    % acceleration vector that determines thrust magnitude and direction.
    h = max(z - consts.L, 0) ;
    dz_ref = -ctrl.vz_max*tanh(h/ctrl.z_slow) ;
    if(ctrl.initial_theta < -ctrl.recovery_initial_gate)
        dz_ref = ctrl.neg_large_vz_scale*dz_ref ;
    end
    if(ctrl.initial_theta > ctrl.recovery_initial_gate)
        dz_ref = ctrl.pos_large_vz_scale*dz_ref ;
    end
    if(abs(dth) > ctrl.high_rate_gate)
        dz_ref = ctrl.high_rate_vz_scale*dz_ref ;
    end

    % Cruise gate: keep a steady descent speed when high enough and stable
    theta_gate = ctrl.cruise_theta_gate ;
    dtheta_gate = ctrl.cruise_dtheta_gate ;
    y_gate = ctrl.cruise_y_gate ;
    if(ctrl.initial_theta_abs > 2.5)
        theta_gate = ctrl.cruise_theta_gate_hi ;
        dtheta_gate = ctrl.cruise_dtheta_gate_hi ;
        y_gate = ctrl.cruise_y_gate_hi ;
    end
    cruise_ok = z > ctrl.cruise_z && abs(th) < theta_gate && ...
                abs(dth) < dtheta_gate && abs(y) < y_gate ;
    if(cruise_ok)
        dz_ref = min(dz_ref, -ctrl.vz_cruise) ;
    else
        % Near-ground descent cap: first to a faster limit, then to the final limit
        land_alpha_fast = sat((ctrl.land_z_fast - z)/ctrl.land_z_fast, 0, 1) ;
        dz_ref = (1 - land_alpha_fast)*dz_ref + land_alpha_fast*max(dz_ref, -ctrl.vz_land_fast) ;
    end
    land_alpha = sat((ctrl.land_z - z)/ctrl.land_z, 0, 1) ;
    dz_ref = (1 - land_alpha)*dz_ref + land_alpha*max(dz_ref, -ctrl.vz_land) ;

    ay_cmd = -ctrl.ky*y - ctrl.kdy*dy ;
    az_raw = ctrl.kz_v*(dz_ref - dz) ;

    if(ctrl.initial_theta < -1.2 && ctrl.initial_theta_abs < ctrl.recovery_initial_gate)
        ay_cmd = ctrl.neg_angle_y_boost*ay_cmd ;
    end
    ay_cmd = sat(ay_cmd, -ctrl.max_ay, ctrl.max_ay) ;

    % The decoupling formula assumes phi = theta + psi tracks phi_d.  When
    % the attitude is still far away, pause the fast descent so the inner
    % loop has time to point the thrust vector before touchdown.
    aT_z_preview = max(sat(az_raw, ctrl.min_az, ctrl.max_az) + consts.g, 0.5) ;
    phi_preview = sat(atan2(-ay_cmd, aT_z_preview), -ctrl.max_phi, ctrl.max_phi) ;
    phi = wrap_angle(th + psi) ;
    phi = phi(1) ;
    % For high-alt-inverted recoveries, use a looser attitude gate so the
    % rocket descends faster once theta is nearly recovered.
    if(ctrl.initial_theta_abs > ctrl.hai_theta_thresh)
        safe_vz_gate_th  = 1.2 ;
        safe_vz_gate_phi = 1.4 ;
    else
        safe_vz_gate_th  = 0.5 ;
        safe_vz_gate_phi = 0.7 ;
    end
    if(abs(wrap_angle(th - phi_preview)) > safe_vz_gate_th || abs(wrap_angle(phi - phi_preview)) > safe_vz_gate_phi)
        dz_ref = max(dz_ref, -ctrl.safe_vz) ;
        az_cmd = sat(ctrl.kz_v*(dz_ref - dz), ctrl.safe_min_az, ctrl.max_az) ;
    else
        az_cmd = sat(az_raw, ctrl.min_az, ctrl.max_az) ;
    end

    large_angle_recovery = abs(th) > ctrl.large_angle_recovery && ...
                           (ctrl.initial_theta_abs > ctrl.recovery_initial_gate || ...
                            abs(dth) > ctrl.high_rate_gate) ;
    large_angle_recovery = large_angle_recovery(1) ;

    % Optional debug logging (once per second)
    persistent last_log_t
    if(isempty(last_log_t))
        last_log_t = -inf ;
    end
    debug_on = isfield(ctrl, 'debug') && ~isempty(ctrl.debug) && ctrl.debug(1) ~= 0 ;
    debug_on = debug_on(1) ;
    t_scalar = t(1) ;
    if(t_scalar < last_log_t)
        last_log_t = -inf ;
    end
    if(debug_on && (t_scalar - last_log_t >= ctrl.debug_dt))
        last_log_t = t_scalar ;
        mode = 'normal' ;
        if(large_angle_recovery)
            mode = 'large_angle' ;
        elseif(cruise_ok)
            mode = 'cruise' ;
        end
        fprintf('t=%.0f mode=%s th=%.2f dth=%.2f y=%.1f z=%.1f\n', ...
                t_scalar, mode, th, dth, y, z) ;
    end

    % High-altitude near-inverted override: z > threshold AND |th| > threshold
    % Also require initial angle was large to avoid triggering on transient swings
    high_alt_inverted = z > ctrl.hai_z_thresh && abs(th) > ctrl.hai_theta_thresh && ...
                        ctrl.initial_theta_abs > ctrl.hai_theta_thresh ;
    % Negative-angle high-altitude recovery: same zone but for negative initial theta
    high_alt_neg = z > ctrl.hai_z_thresh && th <= -ctrl.hai_theta_thresh && ...
                   ctrl.initial_theta <= -ctrl.hai_theta_thresh ;
    % Deep-angle override for very large angles
    deep_angle = abs(th) > ctrl.deep_theta_thresh && z > ctrl.hai_z_thresh ;
    deep_exit = abs(th) < ctrl.deep_theta_exit ;
    persistent spin_active last_mode
    if(isempty(spin_active))
        spin_active = false ;
    end
    if(isempty(last_mode))
        last_mode = 'normal' ;
    end
    spin_active = false ;
    spin_guard = false ;
    spin_exit = true ;
    if(high_alt_inverted || high_alt_neg || deep_angle)
        dz_ref = max(dz_ref, -ctrl.hai_vz_max) ;
        if(deep_angle)
            dz_ref = max(dz_ref, -ctrl.deep_vz_max) ;
        end
        az_cmd = sat(ctrl.kz_v*(dz_ref - dz), ctrl.safe_min_az, ctrl.max_az) ;
    end

    if((deep_angle && ~deep_exit) || (spin_guard && ~spin_exit))
        if(ctrl.deep_ay_zero)
            ay_cmd = 0 ;
        else
            ay_cmd = ctrl.deep_ay_scale*ay_cmd ;
        end
    end

    aT_y = ay_cmd ;
    aT_z = max(az_cmd + consts.g, 0.5) ;

    if(large_angle_recovery)
        phi_d = 0 ;
        if(high_alt_inverted || high_alt_neg || deep_angle || spin_guard)
            psi_limit = ctrl.hai_psi_limit ;
            if(deep_angle)
                psi_limit = ctrl.deep_psi_limit ;
            end
            if(spin_guard)
                psi_limit = ctrl.spin_psi_limit ;
            end
        else
            psi_limit = ctrl.max_psi_cmd ;
        end
        % Use stronger CLF gains for high-alt near-inverted case
        if(high_alt_inverted || high_alt_neg || deep_angle || spin_guard)
            clf_lambda = ctrl.hai_clf_lambda ;
            clf_k      = ctrl.hai_clf_k ;
            if(deep_angle)
                clf_lambda = ctrl.deep_clf_lambda ;
                clf_k      = ctrl.deep_clf_k ;
            end
            if(spin_guard)
                clf_lambda = ctrl.spin_clf_lambda ;
                clf_k      = ctrl.spin_clf_k ;
            end
        else
            clf_lambda = ctrl.clf_lambda ;
            clf_k      = ctrl.clf_k ;
        end
        % Use shortest angular path to zero for CLF
        th_err = wrap_angle(th) ;
        dth_eff = sat(dth, -ctrl.large_dth_cap, ctrl.large_dth_cap) ;
        s_theta = dth_eff + clf_lambda*th_err ;
        ddtheta_clf = -clf_lambda*dth_eff - clf_k*s_theta ;
        ddtheta_clf = sat(ddtheta_clf, -ctrl.large_ddtheta_max, ctrl.large_ddtheta_max) ;
        c_base = consts.L*consts.gamma/(consts.Jm*m) ;
        fT_need = abs(ddtheta_clf)/(c_base*sin(psi_limit)) ;
        if(abs(th) > ctrl.inverted_recovery_angle)
            recovery_cap = ctrl.recovery_fT_inverted ;
            if(high_alt_neg)
                % Negative deeply-inverted: mirror positive strategy
                if(abs(th) > ctrl.hai_fT_switch_angle)
                    recovery_cap = ctrl.hai_fT_deep ;
                else
                    recovery_cap = ctrl.hai_fT_mid ;
                end
            elseif(high_alt_inverted)
                if(th > 0)
                    if(abs(th) > ctrl.hai_fT_switch_angle)
                        recovery_cap = ctrl.hai_fT_deep ;
                    else
                        recovery_cap = ctrl.hai_fT_mid ;
                    end
                end
            end
        elseif(ctrl.initial_theta < -ctrl.inverted_recovery_angle)
            recovery_cap = ctrl.recovery_fT_neg ;
        elseif(y*sin(phi) > 0)
            recovery_cap = ctrl.recovery_fT_help ;
        else
            recovery_cap = ctrl.recovery_fT_hurt ;
        end
        fT = min(recovery_cap, ctrl.large_fT_max) ;
    else
        phi_d = atan2(-aT_y, aT_z) ;
        phi_d = sat(phi_d, -ctrl.max_phi, ctrl.max_phi) ;
        phi_d = phi_d(1) ;
        fT = m/consts.gamma*sqrt(aT_y^2 + aT_z^2) ;
    end

    % Rate-limit desired thrust angle to avoid abrupt attitude changes
    persistent phi_cmd last_phi_t
    if(isempty(phi_cmd) || numel(phi_cmd) ~= 1)
        phi_cmd = phi_d ;
        last_phi_t = t ;
    end
    dt = max(t - last_phi_t, 1e-3) ;
    last_phi_t = t ;
    rate_limit = ctrl.phi_rate_limit ;
    if(spin_guard && ~spin_exit)
        rate_limit = ctrl.spin_phi_rate_limit ;
    end
    if(th < 0)
        rate_limit = ctrl.neg_phi_rate_limit ;
    end
    if(deep_angle)
        rate_limit = ctrl.deep_phi_rate_limit ;
        if(th < 0)
            rate_limit = ctrl.deep_neg_phi_rate_limit ;
        end
    end
    phi_step = sat(phi_d - phi_cmd, -rate_limit*dt, rate_limit*dt) ;
    phi_cmd = phi_cmd + phi_step ;
    phi_d = phi_cmd ;
    angle_err = wrap_angle(phi - phi_d) ;
    angle_err = angle_err(1) ;
    if(~large_angle_recovery && abs(angle_err) > 0.35 && cos(phi) > 0)
        fT = max(fT, m/consts.gamma*aT_z/max(cos(phi), 0.3)) ;
    end

    if(spin_guard && ~spin_exit)
        fT = min(fT, ctrl.spin_fT) ;
    end

    % High-angle debug logging
    persistent last_dbg_t
    if(isempty(last_dbg_t))
        last_dbg_t = -inf ;
    end
    if(ctrl.debug_state_chain)
        mode = 'normal' ;
        if(spin_guard && ~spin_exit)
            mode = 'spin_guard' ;
        elseif(large_angle_recovery)
            mode = 'large_angle' ;
        elseif(cruise_ok)
            mode = 'cruise' ;
        end
        if(~strcmp(last_mode, mode))
            fprintf('t=%.2f transition %s -> %s | th=%.2f dth=%.2f z=%.1f\n', ...
                    t, last_mode, mode, th, dth, z) ;
            last_mode = mode ;
        end
    end
    if(ctrl.debug_high_angle && (spin_guard || large_angle_recovery))
        if(t - last_dbg_t >= 0.25)
            last_dbg_t = t ;
            mode = 'normal' ;
            if(spin_guard && ~spin_exit)
                mode = 'spin_guard' ;
            elseif(large_angle_recovery)
                mode = 'large_angle' ;
            end
            fprintf(['t=%.2f z=%.1f th=%.2f dth=%.2f phi=%.2f phi_d=%.2f ' ...
                     'fT=%.2f mode=%s\n'], ...
                    t, z, th, dth, phi, phi_d, fT, mode) ;
        end
    end

    % ---- Cascade inner loop ----
    % Let theta_d follow the desired thrust direction.  Then use the gimbal
    % as the virtual input that creates the requested angular acceleration.
    theta_d = phi_d ;
    e_theta = wrap_angle(th - theta_d) ;
    ddtheta_d = -ctrl.ktheta*e_theta - ctrl.kdtheta*dth ;

    Ctheta = consts.L*consts.gamma*max(fT, consts.min.fT)/(consts.Jm*m) ;
    psi_limit = ctrl.max_psi_cmd ;
    psi_d_raw = asin(sat(-ddtheta_d/max(Ctheta, 1e-3), -sin(psi_limit), sin(psi_limit))) ;

    integrator_x = x_full(10:end) ;
    if(isempty(integrator_x))
        psi_cmd = psi_d_raw ;
        psi_cmd_dot = 0 ;
    else
        psi_cmd = sat(integrator_x(1), -psi_limit, psi_limit) ;
        psi_cmd_dot = sat(ctrl.psi_filter_gain*wrap_angle(psi_d_raw - psi_cmd), ...
                          -ctrl.max_psi_rate, ctrl.max_psi_rate) ;
    end

    e_psi = wrap_angle(psi - psi_cmd) ;
    tau = consts.JT*(-ctrl.kpsi*e_psi - ctrl.kdpsi*(dpsi - psi_cmd_dot)) ;
    if(spin_guard && ~spin_exit)
        tau = sat(tau, -ctrl.spin_tau_max, ctrl.spin_tau_max) ;
    end

    % Output control input [thrust; torque].  The simulator applies the final
    % actuator saturations; this pre-saturation only avoids numerical spikes.
    u = [sat(fT, consts.min.fT, consts.max.fT);
         sat(tau, -consts.max.tau, consts.max.tau)] ;

    if(ctrl.N_integrators > 0)
        integrator_dx = psi_cmd_dot ;
    else
        integrator_dx = zeros(0,1) ;
    end
end

function y = sat(x, lo, hi)
    y = min(max(x, lo), hi) ;
end

function a = wrap_angle(a)
    a = atan2(sin(a), cos(a)) ;
end
