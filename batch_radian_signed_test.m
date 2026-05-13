function results = batch_radian_signed_test()
    close all;
    consts = get_consts();
    angles = [-2.0 -1.5 -1.3 -1.0 -0.5 0.5 1.0 1.3 1.5 2.0];
    results = zeros(length(angles), 8);
    for k = 1:length(angles)
        x0 = [50; 550; angles(k); 0; 0; 0; 0; 0; consts.m_nofuel + consts.max.m_fuel];
        [ctrl, ~] = student_setup(x0, consts);
        x0_full = [x0; zeros(ctrl.N_integrators, 1)];
        odeopts = odeset('Events', @odeevents_touchdown_local);
        [t, x] = ode45(@odefun_local, [0 60], x0_full, odeopts, 0, consts, ctrl);
        J = compute_score(x(end,1:9)', consts);
        results(k,:) = [angles(k), t(end), J, x(end,1), x(end,2), x(end,3)*180/pi, norm(x(end,5:6)), x(end,7)*180/pi];
        fprintf('angle=%6.2f rad (%7.1f deg)  t=%6.2f  J=%7.2f  y=%9.2f  z=%7.2f  th=%8.3f deg  speed=%7.2f  dth=%8.3f deg/s\n', angles(k), angles(k)*180/pi, t(end), J, x(end,1), x(end,2), x(end,3)*180/pi, norm(x(end,5:6)), x(end,7)*180/pi);
    end
end

function [dx, u] = odefun_local(t, x, wind, consts, ctrl)
    y = x(1); z = x(2); th = x(3); psi = x(4); dy = x(5); dz = x(6); m = x(9);
    consts.J = consts.Jm*m;
    f_vec = [dy; dz; x(7); x(8); 0; -consts.g; 0; 0; 0];
    g_vec = [0,0; 0,0; 0,0; 0,0; -consts.gamma*sin(psi+th)/m,0; consts.gamma*cos(psi+th)/m,0; -consts.L*consts.gamma*sin(psi)/consts.J,0; 0,1/consts.JT; -1,0];
    if(x(2) > 2*consts.L), d = wind/m*exp(-1/(x(2)-2*consts.L)); else, d = 0; end
    [u, integrator_dx] = student_controller(t, x, consts, ctrl);
    if(m <= consts.m_nofuel), u(1) = 0; end
    u(1) = min(max(u(1), consts.min.fT), consts.max.fT); u(2) = min(max(u(2), -consts.max.tau), consts.max.tau);
    dx = [f_vec + g_vec*u + [zeros(4,1); d; zeros(4,1)]; integrator_dx];
end

function [value, isterminal, direction] = odeevents_touchdown_local(t, x, wind, consts, ctrl)
    z = x(2); th = x(3); L = consts.L; r = consts.r;
    value = min([z-L*cos(th)-r*sin(th); z-L*cos(th)+r*sin(th); z+L*cos(th); z-r*sin(th); z+r*sin(th)]);
    isterminal = 1; direction = 0;
end
