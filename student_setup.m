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
    % Replace code below with any one time control design computation if needed.
    % Ex: ctrl.K = lqr(A, B, Q, R) ; (teacher's LQR, Baseline)
    % ctrl.K = [0.0000    0.0018    0.0997    0.4690   -0.0005    0.0353    0.4850    1.8646    0.0000;
    %           0.0000    0.0000   -0.0036    1.0045    0.0000    0.0000   -0.0469    3.8980    0.0000] ;
    ctrl.K = [   -0.0000    2.5000    0.0000   -0.0000   -0.0000    5.0867    0.0000    0.0000   -0.0098
        0.5000    0.0000  -71.4795  120.1597    3.3595   -0.0000  -81.6727   40.1654   -0.0000]

    

    % [Advanced - default should work] We provide possibility to switch the ode solver if needed.
    % Default ode_type should work in almost all cases.  Don't change this unless you know what you are doing.
    ctrl.ode_type = 0 ; % zero => ode45, non-zero=> ode15s.
    
    % [Advanced - default should work] No. of integrators.  These integrators will be added to ode45 states.
    % Your student_controller file will have to supply the derivative (dx) of these additional states.
    % Ode45 will then integrate this for you in addition to the dynamics.
    ctrl.N_integrators = 0 ;
end