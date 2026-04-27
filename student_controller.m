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

% (Teacher's LQR, baseline)
function [u, integrator_dx] = student_controller(t, x_full, consts, ctrl)

    % Extract state
    x = x_full(1:9) ; % The first 9 states corresponding to the rocket states


    % [Sample Controller - REPLACE THIS]
    % Replace the code below with your controller
    % Ex: u = -ctrl.K * x ;
    % A (bad) state-feedback controller is given below.

    m = x(end) ; % Extract Rocket Mass state
    x_d = [0;consts.L;0;0; 0;0;0;0;  m] ; % Specify desired landing state
    u_ff = [m*consts.g / consts.gamma ; 0] ; % Compute feedforward control, cancel gravity

    % Output control input [thrust; torque]
    u = u_ff - ctrl.K*(x - x_d) ; % Compute control


    % [Advanced] Replace this if you are using integrators to provide what you want to integrate.
    integrator_x = x_full(10:end) ; % extracting integrator states
    integrator_dx = zeros(size(integrator_x)) ; % Replace this by what you want integrated
end

