function report = check_recovery_feasibility(x0)
% Feasibility check for the current dynamic-surface backstepping inner loop.
% It estimates whether the theta -> psi_cmd -> tau chain can realize the
% initial attitude correction without immediately demanding impossible gimbal
% motion.  This does not change the simulation.

    consts = get_consts();
    [ctrl, ~] = student_setup(x0, consts);

    th = atan2(sin(x0(3)), cos(x0(3)));
    psi = atan2(sin(x0(4)), cos(x0(4)));
    dth = x0(7);
    dpsi = x0(8);
    m = x0(9);

    fT_hover = min(max(m*consts.g/consts.gamma, consts.min.fT), consts.max.fT);
    Ctheta = consts.L*consts.gamma*fT_hover/(consts.Jm*m);

    % Same virtual-control calculation used by student_controller near hover.
    theta_d = 0;
    e_theta = atan2(sin(th - theta_d), cos(th - theta_d));
    ddtheta_d = -ctrl.ktheta*e_theta - ctrl.kdtheta*dth;
    psi_d_raw = asin(min(max(-ddtheta_d/max(Ctheta, 1e-3), ...
                            -sin(ctrl.max_psi_cmd)), sin(ctrl.max_psi_cmd)));

    psi_cmd0 = 0; % simulator initializes the added filter integrator at zero
    psi_cmd_dot = min(max(ctrl.psi_filter_gain*atan2(sin(psi_d_raw - psi_cmd0), ...
                                                     cos(psi_d_raw - psi_cmd0)), ...
                          -ctrl.max_psi_rate), ctrl.max_psi_rate);

    tau_cmd = consts.JT*(-ctrl.kpsi*atan2(sin(psi - psi_cmd0), cos(psi - psi_cmd0)) - ...
                         ctrl.kdpsi*(dpsi - psi_cmd_dot));

    report.theta_deg = th*180/pi;
    report.psi_deg = psi*180/pi;
    report.dtheta_deg_s = dth*180/pi;
    report.dpsi_deg_s = dpsi*180/pi;
    report.Ctheta = Ctheta;
    report.psi_d_raw_deg = psi_d_raw*180/pi;
    report.psi_cmd_dot_deg_s = psi_cmd_dot*180/pi;
    report.tau_cmd = tau_cmd;
    report.tau_margin = consts.max.tau - abs(tau_cmd);
    report.gimbal_command_feasible = abs(psi_d_raw) <= ctrl.max_psi_cmd;
    report.torque_feasible = abs(tau_cmd) <= consts.max.tau;
    report.inner_loop_feasible = report.gimbal_command_feasible && report.torque_feasible;

    disp(report)
end
