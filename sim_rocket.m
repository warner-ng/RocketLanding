% Function to simulate rocket
function sim_rocket(x0, wind)
    close all ;

    % load constant parameters
    consts = get_consts() ;

    if(nargin < 1)
        x0 = [50; 550; 1; 0;
          0; 0; 0; 0;
          consts.m_nofuel+1.0*consts.max.m_fuel] ;
        wind = 0 ;
    end

    % call student one-time setup
    [ctrl, student_data] = student_setup(x0, consts) ; %#ok<ASGLU>

    % Integrators feature if used (Append integrator states)
    x0 = [x0; zeros(ctrl.N_integrators, 1)] ;

    % Integrate system
    odeopts = odeset('Events',@odeevents_touchdown) ;
    if(ctrl.ode_type == 0)
        [t, x] = ode45(@odefun_rocket, [0 60], x0, odeopts, wind, consts, ctrl) ;
    else
        [t, x] = ode15s(@odefun_rocket, [0 60], x0, odeopts, wind, consts, ctrl) ;
    end


    % Display Helpful information after simulation
    disp(['Touchdown Time: ' num2str(t(end))]) ;
    disp(['Touchdown Configuration: y=' num2str(x(end,1)) ' z=' num2str(x(end,2)) ' theta(deg)=' num2str(x(end,3)*180/pi) ' psi(deg)=' num2str(x(end,4)*180/pi)]) ;
    disp(['Touchdown Velocity: y=' num2str(x(end,5)) ' z=' num2str(x(end,6)) ' theta(deg/s)=' num2str(x(end,7)*180/pi) ' psi(deg/s)=' num2str(x(end,8)*180/pi)]) ;

    J = compute_score(x(end,:)', consts) ;
    disp(['Score: ' num2str(J)]) ;

    % --------------------------------------------------------------
    % Post-processing: re-call controller at every saved step to
    % recover (a) the input u(t) and (b) the diagnostic struct.
    % --------------------------------------------------------------
    N = length(t) ;
    u = zeros(N, 2) ;

    have_diag = isfield(ctrl, 'want_diag') && ctrl.want_diag && ...
                (nargout(@student_controller) >= 3) ;

    if have_diag
        diag.phase       = zeros(N,1) ;
        diag.cphi        = zeros(N,1) ;
        diag.fT          = zeros(N,1) ;
        diag.fT_max_eff  = zeros(N,1) ;
        diag.fT_hover    = zeros(N,1) ;
        diag.tau         = zeros(N,1) ;
        diag.theta_d     = zeros(N,1) ;
        diag.th_w        = zeros(N,1) ;
        diag.psi_cmd     = zeros(N,1) ;
        diag.v_z         = zeros(N,1) ;
        diag.zdot_target = zeros(N,1) ;
        diag.fuel        = zeros(N,1) ;
        diag.arg_arcsin  = zeros(N,1) ;
    end

    % NOTE: phase has internal `persistent` state in the controller; clear
    % it so the post-processing pass re-derives the same phase trajectory.
    clear student_controller ;

    for j = 1:N
        if have_diag
            [uu, ~, dj] = student_controller(t(j), x(j,:)', consts, ctrl) ;
            diag.phase(j)       = dj.phase ;
            diag.cphi(j)        = dj.cphi ;
            diag.fT(j)          = dj.fT ;
            diag.fT_max_eff(j)  = dj.fT_max_eff ;
            diag.fT_hover(j)    = dj.fT_hover ;
            diag.tau(j)         = dj.tau ;
            diag.theta_d(j)     = dj.theta_d ;
            diag.th_w(j)        = dj.th_w ;
            diag.psi_cmd(j)     = dj.psi_cmd ;
            diag.v_z(j)         = dj.v_z ;
            diag.zdot_target(j) = dj.zdot_target ;
            diag.fuel(j)        = dj.fuel ;
            diag.arg_arcsin(j)  = dj.arg_arcsin ;
        else
            [~, uu] = odefun_rocket(t(j), x(j,:)', wind, consts, ctrl) ;
        end
        u(j,:) = uu' ;
    end

    % --------------------------------------------------------------
    % State plots (kept from original)
    % --------------------------------------------------------------
    figure('Name','State') ;
    subplot(2,2,1); plot(t, x(:,1)); grid on; xlabel('t (s)'); ylabel('y (m)');     title('horizontal pos') ;
    subplot(2,2,2); plot(t, x(:,2)); grid on; xlabel('t (s)'); ylabel('z (m)');     title('altitude') ;
    subplot(2,2,3); plot(t, x(:,3)*180/pi); grid on; xlabel('t (s)'); ylabel('\theta (deg)'); title('attitude') ;
    subplot(2,2,4); plot(t, x(:,4)*180/pi); grid on; xlabel('t (s)'); ylabel('\psi (deg)');   title('gimbal') ;

    % --------------------------------------------------------------
    % Diagnostic plots (NEW)
    %   These are the "intermediate variables" used during design
    %   validation: they tell you WHY a score is low, not just THAT it is.
    % --------------------------------------------------------------
    if have_diag
        figure('Name','Diagnostics: actuators, phase, cos(\theta+\psi)') ;
        subplot(3,2,1) ;
            plot(t, u(:,1), 'b', t, diag.fT_max_eff, 'r--', t, diag.fT_hover, 'g:') ;
            grid on; xlabel('t'); ylabel('f_T (N)') ;
            legend('f_T applied','f_T^{max,eff}','hover','Location','best') ;
            title('Thrust (saturation = z-channel design problem)') ;
        subplot(3,2,2) ;
            plot(t, u(:,2), 'b', t, +consts.max.tau*ones(size(t)), 'r--', ...
                              t, -consts.max.tau*ones(size(t)), 'r--') ;
            grid on; xlabel('t'); ylabel('\tau (N m)') ;
            title('Gimbal torque') ;
        subplot(3,2,3) ;
            plot(t, diag.phase, 'k', 'LineWidth', 1.5) ;
            grid on; xlabel('t'); ylabel('phase'); ylim([0.5 3.5]) ;
            yticks([1 2 3]); yticklabels({'P1 recover','P2 descent','P3 final'}) ;
            title('Phase (must exit P1 quickly)') ;
        subplot(3,2,4) ;
            plot(t, diag.cphi, 'b', t, ctrl.cos_thresh_lo*ones(size(t)),'r--', ...
                                  t, ctrl.cos_thresh_hi*ones(size(t)),'g--') ;
            grid on; xlabel('t'); ylabel('cos(\theta+\psi)') ;
            legend('cos(\phi)','enter P1','leave P1','Location','best') ;
            title('Decoupling-matrix conditioning') ;
        subplot(3,2,5) ;
            plot(t, diag.fuel, 'b') ;
            grid on; xlabel('t'); ylabel('fuel left (kg)') ;
            title(sprintf('Fuel (used %.1f kg of %.0f)', ...
                  diag.fuel(1)-diag.fuel(end), diag.fuel(1))) ;
        subplot(3,2,6) ;
            plot(t, x(:,9), 'b') ;
            grid on; xlabel('t'); ylabel('m (kg)') ;
            title('Total mass') ;

        figure('Name','Diagnostics: cascade tracking') ;
        subplot(3,1,1) ;
            plot(t, x(:,3)*180/pi, 'b', t, diag.theta_d*180/pi, 'r--') ;
            grid on; xlabel('t'); ylabel('deg') ;
            legend('\theta','\theta_{desired}','Location','best') ;
            title('Outer-y -> theta tracking (error -> y oscillates)') ;
        subplot(3,1,2) ;
            plot(t, x(:,4)*180/pi, 'b', t, diag.psi_cmd*180/pi, 'r--', ...
                 t,  ctrl.psi_cmd_max*180/pi*ones(size(t)),'k:', ...
                 t, -ctrl.psi_cmd_max*180/pi*ones(size(t)),'k:') ;
            grid on; xlabel('t'); ylabel('deg') ;
            legend('\psi','\psi_{cmd}','cap','Location','best') ;
            title('Theta -> psi cascade (cap-clamped means cascade saturated)') ;
        subplot(3,1,3) ;
            plot(t, diag.arg_arcsin, 'b', ...
                 t,  ctrl.arcsin_clamp*ones(size(t)),'r--', ...
                 t, -ctrl.arcsin_clamp*ones(size(t)),'r--') ;
            grid on; xlabel('t'); ylabel('arcsin arg') ;
            title('arcsin clamp activations (means torque demand exceeds capacity)') ;

        figure('Name','Diagnostics: z-channel') ;
        subplot(2,1,1) ;
            plot(t, x(:,2), 'b') ;
            grid on; xlabel('t'); ylabel('z (m)') ; title('altitude') ;
        subplot(2,1,2) ;
            plot(t, x(:,6), 'b', t, diag.zdot_target, 'r--') ;
            grid on; xlabel('t'); ylabel('m/s') ;
            legend('zdot','zdot_{target}(z)','Location','best') ;
            title('Vertical velocity tracking') ;
    end

    % Animation
    animate_rocket(t, x, u) ;
end


% Function to describe the dynamics of the rocket
function [dx u] = odefun_rocket(t, x, wind, consts, ctrl)
    % Extract various states
    y = x(1) ;
    z = x(2) ;
    th = x(3) ;
    psi = x(4) ;

    dy = x(5) ;
    dz = x(6) ;
    dth = x(7) ;
    dpsi = x(8) ;

    m = x(9) ;
    consts.J = consts.Jm*m ;

    % Construct drift and control vector fields
    f_vec = [   dy
               dz
              dth
             dpsi
                0
               -consts.g
                0
                0
                0] ;

    g_vec = [0,    0 ;
             0,    0 ;
             0,    0 ;
             0,    0 ;
            -consts.gamma*sin(psi+th)/m,    0 ;
             consts.gamma*cos(psi+th)/m,    0 ;
             -consts.L*consts.gamma*sin(psi)/consts.J,    0 ;
             0, 1/consts.JT ;
            -1,    0] ;

    % Wind disturbance goes smoothly to zero for z <= 2*L
    if(z > 2*consts.L)
        d = wind/m*exp(-1/(z-2*consts.L)) ;
    else
        d = 0 ;
    end
    d_vec = [zeros(4,1);
             d;
             zeros(4,1)] ;

    % call student controller
    [u, integrator_dx] = student_controller(t, x, consts, ctrl) ;

    % Check if fuel is over
    if(m <= consts.m_nofuel)
        u(1) = 0 ;
    end

    % Thrust / Torque saturations
    u(1) = min(max(u(1), consts.min.fT), consts.max.fT) ;
    u(2) = min(max(u(2), -consts.max.tau), consts.max.tau) ;

    % Output time-derivative of state
    dx = f_vec + g_vec*u + d_vec ;

    dx = [dx; integrator_dx] ;
end


% Exit integration on ground contact
function [value,isterminal,direction] = odeevents_touchdown(t, x, wind, consts, ctrl)
    z = x(2) ; th = x(3) ; L = consts.L ; r = consts.r ;
    % Make a list of possible contact points
    contact_points = [z-L*cos(th)-r*sin(th);
                      z-L*cos(th)+r*sin(th);
                      z+L*cos(th);
                      z-r*sin(th);
                      z+r*sin(th)] ;
    value = min(contact_points) ;
    isterminal = 1;   % Stop the integration
    direction = 0;   % all direction only
end
