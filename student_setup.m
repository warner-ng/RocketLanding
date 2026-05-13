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
