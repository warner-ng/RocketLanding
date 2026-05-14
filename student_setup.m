% Function to setup and perform any one-time computations
% Input parameters
%   x0 - state of the rocket, x=[y,z,th,psi,dy,dz,dth,dpsi,m]^T
%   consts - structure that contains various system constants
% Output parameters
%   ctrl  -  any student defined control parameters
% student_data - required info to register your score for leaderboard and class grade
function [ctrl, student_data] = student_setup(x0, consts)
    
    % [One time Setup] Provide student information.  
    % Enter your student id and your nick_name.
    % Your student id will be used to register your score for your grade.
    % Your nick_name will be displayed on the leaderboard - this will serve to anonymize you to your peers.  (Choose a wise / cheeky nick name.)
    student_data.id = 3042103210 ;         % UPDATE THIS
    student_data.nick_name = 'Warner Wu' ; % FILL THIS


    % [Controller Setup]
    % Section 51: MIMO feedback linearization / decoupling for outputs
    % h(x) = [y; z].  The outer loop chooses desired COM accelerations,
    % then inverts the thrust magnitude and thrust direction.
    ctrl.method = 'mimo_fbl_cascade' ;

    % Outer-loop acceleration commands.
    ctrl.ky = 0.032 ;
    ctrl.kdy = 0.32 ;
    ctrl.kz_v = 0.60 ;
    ctrl.vz_max = 30.0 ;
    ctrl.z_slow = 100.0 ;
    ctrl.safe_vz = 1.0 ;

    % Cruise gate: keep a steady descent speed 
    % when attitude is stable and higher than ctrl.cruise_z
    ctrl.cruise_z = 150.0 ;
    ctrl.vz_cruise = 60.0 ; 


    ctrl.cruise_theta_gate = 0.7 ;
    ctrl.cruise_dtheta_gate = 1.2 ;
    ctrl.cruise_y_gate = 20.0 ;

    % if not in cruise, near-ground descent caps
    ctrl.land_z_fast = 600.0 ;
    ctrl.vz_land_fast = 8.0 ;

    % when to decelerate to low speed for landing
    ctrl.land_z = 20 ; % basically DO NOT NEED TO CHANGE
    ctrl.vz_land = 4.0 ;

    % Debug logging (set to 1 to print every debug_dt seconds)
    ctrl.debug = 0 ;
    ctrl.debug_dt = 1.0 ;

    % Keep the virtual commands inside the physical thrust cone.
    ctrl.max_ay = 5.0 ;
    ctrl.min_az = -3.0 ;
    ctrl.safe_min_az = 0.0 ;
    ctrl.max_az = 30.0 ;
    ctrl.max_phi = 50*pi/180 ;
    ctrl.large_angle_recovery = 1.15 ;
    ctrl.recovery_initial_gate = 1.40 ;
    ctrl.initial_theta = x0(3) ;
    ctrl.initial_theta_abs = abs(x0(3)) ;
    ctrl.initial_dtheta_abs = abs(x0(7)) ;
    ctrl.high_rate_vz_scale = 0.65 ;
    ctrl.high_rate_gate = 1.2 ;
    ctrl.neg_angle_y_boost = 1.35 ;
    ctrl.neg_large_vz_scale = 0.65 ;
    ctrl.pos_large_vz_scale = 0.65 ;
    ctrl.recovery_fT_help = 0.9 ;
    ctrl.recovery_fT_hurt = 0.30 ;
    ctrl.recovery_fT_neg = 0.55 ;
    ctrl.recovery_fT_inverted = 0.65 ;
    ctrl.inverted_recovery_angle = 1.8 ;
    ctrl.clf_lambda = 1.0 ;
    ctrl.clf_k = 1.3 ;
    ctrl.clf_fT_max = 0.9 ;
    ctrl.clf_lat_accel_max = 5.0 ;

    % High-altitude near-inverted override (z > 1000m AND |th| > 2.2 rad)
    ctrl.hai_z_thresh     = 1000.0 ;    % altitude threshold (m)
    ctrl.hai_theta_thresh = 2.2 ;      % angle threshold (rad)
    ctrl.hai_vz_max       = 6.0 ;      % max descent speed during recovery (m/s)
    ctrl.hai_clf_lambda   = 1.5 ;      % stronger sliding-mode slope
    ctrl.hai_clf_k        = 3.0 ;      % stronger convergence gain
    % fT for near-truly-inverted (|th|>2.7): can use higher thrust safely
    % When |th| > hai_fT_switch_angle (>pi/2), thrust points downward: use minimum
    ctrl.hai_fT_switch_angle = pi/2 ;  % ~1.57 rad: above this thrust is mostly downward
    ctrl.hai_fT_deep      = 0.15 ;     % near min.fT for positive angle deeply inverted
    ctrl.hai_fT_mid       = 1.8 ;      % moderate fT when theta is recoverable (positive)
    ctrl.hai_fT_neg       = 2.5 ;      % fT for negative angle hai recovery (torque generation)
    ctrl.hai_psi_limit    = 40*pi/180 ; % wider gimbal range during hai recovery

    % Inner-loop attitude/gimbal cascade.  The outer loop provides the
    % desired thrust direction phi_d = theta_d; psi is used as a virtual
    % control for theta, and tau tracks psi_d.
    ctrl.ktheta = 1.2 ;
    ctrl.kdtheta = 2.5 ;
    ctrl.kpsi = 80.0 ;
    ctrl.kdpsi = 18.0 ;
    ctrl.max_psi_cmd = 25*pi/180 ;
    ctrl.psi_filter_gain = 4.0 ;
    ctrl.max_psi_rate = 35*pi/180 ;

    % [Advanced - default should work] We provide possibility to switch the ode solver if needed.
    % Default ode_type should work in almost all cases.  Don't change this unless you know what you are doing.
    ctrl.ode_type = 0 ; % zero => ode45, non-zero=> ode15s.
    
    % [Advanced - default should work] No. of integrators.  These integrators will be added to ode45 states.
    % Your student_controller file will have to supply the derivative (dx) of these additional states.
    % Ode45 will then integrate this for you in addition to the dynamics.
    ctrl.N_integrators = 1 ;
end
