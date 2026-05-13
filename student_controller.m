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

    ay_cmd = -ctrl.ky*y - ctrl.kdy*dy ;
    az_raw = ctrl.kz_v*(dz_ref - dz) ;

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

    aT_y = ay_cmd ;
    aT_z = max(az_cmd + consts.g, 0.5) ;

    phi_d = atan2(-aT_y, aT_z) ;
    phi_d = sat(phi_d, -ctrl.max_phi, ctrl.max_phi) ;

    fT = m/consts.gamma*sqrt(aT_y^2 + aT_z^2) ;
    if(abs(wrap_angle(phi - phi_d)) > 0.35 && cos(phi) > 0)
        fT = max(fT, m/consts.gamma*aT_z/max(cos(phi), 0.3)) ;
    end

    % ---- Cascade inner loop ----
    % Let theta_d follow the desired thrust direction.  Then use the gimbal
    % as the virtual input that creates the requested angular acceleration.
    theta_d = phi_d ;
    e_theta = wrap_angle(th - theta_d) ;
    ddtheta_d = -ctrl.ktheta*e_theta - ctrl.kdtheta*dth ;

    Ctheta = consts.L*consts.gamma*max(fT, consts.min.fT)/(consts.Jm*m) ;
    psi_d = asin(sat(-ddtheta_d/max(Ctheta, 1e-3), -sin(ctrl.max_psi_cmd), sin(ctrl.max_psi_cmd))) ;

    e_psi = wrap_angle(psi - psi_d) ;
    tau = consts.JT*(-ctrl.kpsi*e_psi - ctrl.kdpsi*dpsi) ;

    % Output control input [thrust; torque].  The simulator applies the final
    % actuator saturations; this pre-saturation only avoids numerical spikes.
    u = [sat(fT, consts.min.fT, consts.max.fT);
         sat(tau, -consts.max.tau, consts.max.tau)] ;

    % [Advanced] Replace this if you are using integrators to provide what you want to integrate.
    integrator_x = x_full(10:end) ; % extracting integrator states
    integrator_dx = zeros(size(integrator_x)) ; % Replace this by what you want integrated
end

function y = sat(x, lo, hi)
    y = min(max(x, lo), hi) ;
end

function a = wrap_angle(a)
    a = atan2(sin(a), cos(a)) ;
end
