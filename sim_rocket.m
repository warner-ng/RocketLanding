% Function to simulate rocket
function varargout = sim_rocket(x0, wind, opts)
    close all ;

    % load constant parameters
    consts = get_consts() ;


    %   x - state of the rocket, x=[y,z,th,psi,
    %                               dy,dz,dth,dpsi,
    %                               m]
    if(nargin < 1)
        x0 = [50; 1400; 2.85; 0;
          0; 0; 0; 0;
          consts.m_nofuel+1.0*consts.max.m_fuel] ;
        wind = 9 ;
    end
    if(nargin < 3 || isempty(opts))
        opts.verbose = true ;
        opts.do_plots = true ;
        opts.do_anim = true ;
    end
    if(~isfield(opts, 'verbose'))
        opts.verbose = true ;
    end
    if(~isfield(opts, 'do_plots'))
        opts.do_plots = true ;
    end
    if(~isfield(opts, 'do_anim'))
        opts.do_anim = true ;
    end
    
    % call student one-time setup
    ctrl = student_setup(x0, consts) ;
    
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
    J = compute_score(x(end,:)', consts) ;
    if(opts.verbose)
        disp(['Touchdown Time: ' num2str(t(end))]) ;
        disp(['Touchdown Configuration: y=' num2str(x(end,1)) ' z=' num2str(x(end,2)) ' theta(deg)=' num2str(x(end,3)*180/pi) ' psi(deg)=' num2str(x(end,4)*180/pi)]) ;
        disp(['Touchdown Velocity: y=' num2str(x(end,5)) ' z=' num2str(x(end,6)) ' theta(deg/s)=' num2str(x(end,7)*180/pi) ' psi(deg/s)=' num2str(x(end,8)*180/pi)]) ;
        disp(['Score: ' num2str(J)]) ;
    end
    
    if(opts.do_plots)
        figure ; plot(t, x(:,1)) ; grid on ; xlabel('Time (s)') ; ylabel('y (m)') ; title('Rocket Horizontal Position') ;
        figure ; plot(t, x(:,2)) ; grid on ; xlabel('Time (s)') ; ylabel('z (m)') ; title('Rocket Altitude') ;
        figure ; plot(t, x(:,3)*180/pi) ; grid on ; xlabel('Time (s)') ; ylabel('theta (deg)') ;  title('Rocket Attitude') ;
        figure ; plot(t, x(:,4)*180/pi) ; grid on ; xlabel('Time (s)') ; ylabel('psi (deg)') ; title('Rocket Gimbal Angle') ;
    end
    
    if(opts.do_anim)
        u = zeros(length(t), 2) ;
        for j=1:length(t)
            [dx uu] = odefun_rocket(t, x(j,:)', wind, consts, ctrl) ;
            u(j,:) = uu' ;
        end
        animate_rocket(t, x, u) ;
    end

    if(nargout > 0)
        varargout{1} = t ;
    end
    if(nargout > 1)
        varargout{2} = x ;
    end
    if(nargout > 2)
        varargout{3} = J ;
    end
end


% Function to describe the dynamics of the rocket
function [dx u] = odefun_rocket(t, x, wind, consts, ctrl)
    % Ensure state is a column vector (avoid vectorized input issues)
    if(size(x, 2) > 1)
        x = x(:, 1) ;
    end

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

    % Ensure u is a 2x1 column vector for matrix multiplication
    u = u(:) ;
    if(numel(u) < 2)
        u = [u; 0] ;
    elseif(numel(u) > 2)
        u = u(1:2) ;
    end

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