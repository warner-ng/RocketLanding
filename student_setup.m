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
    ctrl.ky = 0.035 ;
    ctrl.kdy = 0.35 ;
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

    % Inner-loop attitude/gimbal cascade.  The outer loop provides the
    % desired thrust direction phi_d = theta_d; psi is used as a virtual
    % control for theta, and tau tracks psi_d.
    ctrl.ktheta = 1.2 ;
    ctrl.kdtheta = 2.5 ;
    ctrl.kpsi = 80.0 ;
    ctrl.kdpsi = 18.0 ;
    ctrl.max_psi_cmd = 25*pi/180 ;

    % [Advanced - default should work] We provide possibility to switch the ode solver if needed.
    % Default ode_type should work in almost all cases.  Don't change this unless you know what you are doing.
    ctrl.ode_type = 0 ; % zero => ode45, non-zero=> ode15s.
    
    % [Advanced - default should work] No. of integrators.  These integrators will be added to ode45 states.
    % Your student_controller file will have to supply the derivative (dx) of these additional states.
    % Ode45 will then integrate this for you in addition to the dynamics.
    ctrl.N_integrators = 0 ;
end
