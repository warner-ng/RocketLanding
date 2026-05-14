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
    cruise_ok = z > ctrl.cruise_z && abs(th) < ctrl.cruise_theta_gate && ...
                abs(dth) < ctrl.cruise_dtheta_gate && abs(y) < ctrl.cruise_y_gate ;
    if(cruise_ok)
        dz_ref = min(dz_ref, -ctrl.vz_cruise) ;
    else
        % Near-ground descent cap: first to a faster limit, then to the final limit
        land_alpha_fast = sat((ctrl.land_z_fast - z)/ctrl.land_z_fast, 0, 1) ;
        dz_ref = (1 - land_alpha_fast)*dz_ref + land_alpha_fast*max(dz_ref, -ctrl.vz_land_fast) ;
    end
    land_alpha = sat((ctrl.land_z - z)/ctrl.land_z, 0, 1) ;
    dz_ref = (1 - land_alpha)*dz_ref + land_alpha*max(dz_ref, -ctrl.vz_land) ;

    % Optional debug logging
    persistent last_log_t
    if(isempty(last_log_t))
        last_log_t = -inf ;
    end
     if(isfield(ctrl, 'debug') && ~isempty(ctrl.debug) && ctrl.debug(1) && ...
         t - last_log_t >= ctrl.debug_dt)
        last_log_t = t ;
        fprintf('t=%.1f z=%.1f dz=%.2f dz_ref=%.2f cruise=%d land_a=%.2f\n', ...
                t, z, dz, dz_ref, cruise_ok, land_alpha) ;
    end

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
    if(abs(wrap_angle(th - phi_preview)) > 0.5 || abs(wrap_angle(phi - phi_preview)) > 0.7)
        dz_ref = max(dz_ref, -ctrl.safe_vz) ;
        az_cmd = sat(ctrl.kz_v*(dz_ref - dz), ctrl.safe_min_az, ctrl.max_az) ;
    else
        az_cmd = sat(az_raw, ctrl.min_az, ctrl.max_az) ;
    end

    large_angle_recovery = abs(th) > ctrl.large_angle_recovery && ...
                           (ctrl.initial_theta_abs > ctrl.recovery_initial_gate || ...
                            abs(dth) > ctrl.high_rate_gate) ;

    aT_y = ay_cmd ;
    aT_z = max(az_cmd + consts.g, 0.5) ;

    if(large_angle_recovery)
        phi_d = 0 ;
        psi_limit = ctrl.max_psi_cmd ;
        s_theta = dth + ctrl.clf_lambda*th ;
        ddtheta_clf = -ctrl.clf_lambda*dth - ctrl.clf_k*s_theta ;
        c_base = consts.L*consts.gamma/(consts.Jm*m) ;
        fT_need = abs(ddtheta_clf)/(c_base*sin(psi_limit)) ;
        if(abs(th) > ctrl.inverted_recovery_angle)
            recovery_cap = ctrl.recovery_fT_inverted ;
        elseif(ctrl.initial_theta < -ctrl.inverted_recovery_angle)
            recovery_cap = ctrl.recovery_fT_neg ;
        elseif(y*sin(phi) > 0)
            recovery_cap = ctrl.recovery_fT_help ;
        else
            recovery_cap = ctrl.recovery_fT_hurt ;
        end
        fT = recovery_cap ;
    else
        phi_d = atan2(-aT_y, aT_z) ;
        phi_d = sat(phi_d, -ctrl.max_phi, ctrl.max_phi) ;
        fT = m/consts.gamma*sqrt(aT_y^2 + aT_z^2) ;
    end
    if(~large_angle_recovery && abs(wrap_angle(phi - phi_d)) > 0.35 && cos(phi) > 0)
        fT = max(fT, m/consts.gamma*aT_z/max(cos(phi), 0.3)) ;
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
