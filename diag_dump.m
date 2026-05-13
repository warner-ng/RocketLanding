% Headless diagnostic dump: run sim_rocket logic but print time series
% summary instead of plotting.  Saves a CSV for later inspection.
function diag_dump(x0, wind)
    consts = get_consts() ;
    if nargin < 1
        x0 = [50; 350; 1; 0;  0; 0; 0; 0;
              consts.m_nofuel + 1.0*consts.max.m_fuel] ;
        wind = 0 ;
    end

    [ctrl, ~] = student_setup(x0, consts) ;

    % Integrate
    % Use ode45 default tolerances to match sim_rocket.m / the grader.
    odeopts = odeset('Events', @(tt,xx) local_touchdown_event(tt, xx, consts)) ;
    [t, x] = ode45(@(tt,xx) local_dyn(tt, xx, wind, consts, ctrl), [0 60], x0, odeopts) ;

    % Re-call controller to collect diagnostics
    clear student_controller
    N = length(t) ;
    fT = zeros(N,1); tau = zeros(N,1); phase = zeros(N,1);
    cphi = zeros(N,1); psi_cmd = zeros(N,1); theta_d = zeros(N,1);
    zdot_tgt = zeros(N,1); v_z = zeros(N,1); arg_arc = zeros(N,1);
    fT_max_eff = zeros(N,1); fuel = zeros(N,1);
    for j = 1:N
        [u, ~, dj] = student_controller(t(j), x(j,:)', consts, ctrl) ;
        fT(j) = u(1); tau(j) = u(2);
        phase(j) = dj.phase; cphi(j) = dj.cphi;
        psi_cmd(j) = dj.psi_cmd; theta_d(j) = dj.theta_d;
        zdot_tgt(j) = dj.zdot_target; v_z(j) = dj.v_z;
        arg_arc(j) = dj.arg_arcsin; fT_max_eff(j) = dj.fT_max_eff;
        fuel(j) = dj.fuel;
    end

    % Summary table -- print at coarse intervals
    fprintf('\n=== Time-series summary ===\n') ;
    fprintf('   t      z      y    th(d) ps(d)  zdot   ydot  | fT  tau  ph  cos  th_d ps_cmd arc fuel\n') ;
    sample_idx = unique(round(linspace(1, N, min(40,N)))) ;
    for k = sample_idx
        fprintf('%5.2f %6.1f %6.1f %5.1f %5.1f %6.1f %6.1f | %4.1f %5.1f  %d %4.2f %5.1f %5.1f %5.2f %5.1f\n', ...
          t(k), x(k,2), x(k,1), x(k,3)*180/pi, x(k,4)*180/pi, x(k,6), x(k,5), ...
          fT(k), tau(k), phase(k), cphi(k), theta_d(k)*180/pi, psi_cmd(k)*180/pi, arg_arc(k), fuel(k)) ;
    end

    % Saturation flags
    fT_saturated = sum(fT >= fT_max_eff - 0.01 & fT_max_eff > 0.5) ;
    fT_at_min    = sum(fT <= ctrl.fT_min + 0.01) ;
    arc_clamped  = sum(abs(arg_arc) >= ctrl.arcsin_clamp - 1e-3) ;
    psi_cmd_clamped = sum(abs(psi_cmd) >= ctrl.psi_cmd_max - 1e-3) ;
    th_d_clamped = sum(abs(theta_d) >= ctrl.theta_des_max_mid - 1e-3) ;
    p1_steps = sum(phase == 1); p3_steps = sum(phase == 3);

    fprintf('\n=== Saturation report (out of %d steps) ===\n', N) ;
    fprintf('  fT at fT_max_eff      : %d (%.0f%%)\n', fT_saturated, 100*fT_saturated/N) ;
    fprintf('  fT at fT_min (0.1)    : %d (%.0f%%)\n', fT_at_min,    100*fT_at_min/N) ;
    fprintf('  arcsin arg clamped    : %d (%.0f%%)\n', arc_clamped,  100*arc_clamped/N) ;
    fprintf('  psi_cmd capped at max : %d (%.0f%%)\n', psi_cmd_clamped, 100*psi_cmd_clamped/N) ;
    fprintf('  theta_d capped at max : %d (%.0f%%)\n', th_d_clamped, 100*th_d_clamped/N) ;
    fprintf('  Phase 1 steps         : %d (%.0f%%)\n', p1_steps, 100*p1_steps/N) ;
    fprintf('  Phase 3 steps         : %d (%.0f%%)\n', p3_steps, 100*p3_steps/N) ;

    fprintf('\n=== Final state ===\n') ;
    fprintf('  t = %.2f, y=%.2f, z=%.2f, theta=%.2f deg, psi=%.2f deg\n', ...
        t(end), x(end,1), x(end,2), x(end,3)*180/pi, x(end,4)*180/pi) ;
    fprintf('  ydot=%.2f, zdot=%.2f, thdot=%.2f deg/s, psidot=%.2f deg/s\n', ...
        x(end,5), x(end,6), x(end,7)*180/pi, x(end,8)*180/pi) ;
    fprintf('  speed = %.2f m/s, fuel used = %.2f kg\n', ...
        norm(x(end,5:6)), fuel(1)-fuel(end)) ;
    fprintf('  Score = %.2f\n', compute_score(x(end,:)', consts)) ;
end

function dx = local_dyn(t, x, wind, consts, ctrl)
    consts.J = consts.Jm * x(9) ;
    f_vec = [x(5); x(6); x(7); x(8); 0; -consts.g; 0; 0; 0] ;
    g_vec = [0,0; 0,0; 0,0; 0,0;
             -consts.gamma*sin(x(4)+x(3))/x(9), 0;
              consts.gamma*cos(x(4)+x(3))/x(9), 0;
             -consts.L*consts.gamma*sin(x(4))/consts.J, 0;
              0, 1/consts.JT;
             -1, 0] ;
    if x(2) > 2*consts.L
        d = wind / x(9) * exp(-1/(x(2)-2*consts.L));
    else
        d = 0;
    end
    d_vec = [zeros(4,1); d; zeros(4,1)] ;
    [u, ~] = student_controller(t, x, consts, ctrl) ;
    if x(9) <= consts.m_nofuel, u(1) = 0 ; end
    u(1) = min(max(u(1), consts.min.fT), consts.max.fT) ;
    u(2) = min(max(u(2), -consts.max.tau), consts.max.tau) ;
    dx = f_vec + g_vec*u + d_vec ;
end

function [v,is,d] = local_touchdown_event(t, x, consts) %#ok<INUSL>
    L = consts.L ; r = consts.r ; th = x(3) ; z = x(2) ;
    cps = [z-L*cos(th)-r*sin(th); z-L*cos(th)+r*sin(th);
           z+L*cos(th); z-r*sin(th); z+r*sin(th)] ;
    v = min(cps); is = 1; d = 0;
end
