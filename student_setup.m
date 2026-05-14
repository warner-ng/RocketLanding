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
    ctrl.debug_high_angle = 1 ; % 高角度调试日志开关

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

    % Large-angle stabilization (merged behavior)
    ctrl.large_dth_cap      = 1.0 ;   % 大角度时角速度限幅 (rad/s)
    ctrl.large_ddtheta_max  = 0.8 ;   % 大角度时角加速度限幅 (rad/s^2)
    ctrl.large_fT_max       = 0.8 ;   % 大角度时推力上限

    % 高空大角度恢复策略（z > 阈值 且 |th| > 阈值）
    ctrl.hai_z_thresh     = 600.0 ;    % 高度阈值 (m)
    ctrl.hai_theta_thresh = 1.9 ;      % 角度阈值 (rad)
    ctrl.hai_vz_max       = 4.0 ;      % 恢复时允许的最大下降速度 (m/s)
    ctrl.hai_clf_lambda   = 2.0 ;      % CLF 斜率系数（更强）
    ctrl.hai_clf_k        = 4.0 ;      % CLF 收敛系数（更强）
    % 反转区间推力分段：超过切换角时，推力主要向下，取更小值
    ctrl.hai_fT_switch_angle = pi/2 ;  % 角度切换点 (rad)，约 1.57
    ctrl.hai_fT_deep      = 0.35 ;     % 深度反转区推力 (|th| > pi/2)
    ctrl.hai_fT_mid       = 2.2 ;      % 可恢复区推力（正角度）
    ctrl.hai_fT_neg       = 3.0 ;      % 负角度恢复推力（便于产生扭矩）
    ctrl.hai_psi_limit    = 50*pi/180 ; % 高空恢复时更大的摆角限制

    % 深度反转补偿（|th| > 2.5 rad）
    ctrl.deep_theta_thresh = 2.5 ;     % 深度反转角度阈值 (rad)
    ctrl.deep_theta_exit   = 2.2 ;     % 深度反转退出阈值 (rad)
    ctrl.deep_vz_max       = 2.0 ;     % 深度反转时最大下降速度 (m/s)
    ctrl.deep_clf_lambda   = 3.0 ;     % 深度反转 CLF 斜率系数
    ctrl.deep_clf_k        = 6.0 ;     % 深度反转 CLF 收敛系数
    ctrl.deep_psi_limit    = 70*pi/180 ; % 深度反转时摆角限制
    ctrl.deep_ay_scale     = 0.2 ;     % 深度反转时横向加速度缩放
    ctrl.deep_ay_zero       = 1 ;      % 深度反转模式: 1 表示横向加速度归零

    % 期望倾斜角变化速率限制（抑制大角度抖动）
    ctrl.phi_rate_limit    = 45*pi/180 ; % 常规期望倾斜角变化速率 (rad/s)
    ctrl.deep_phi_rate_limit = 25*pi/180 ; % 深度反转时更慢的变化速率 (rad/s)
    ctrl.neg_phi_rate_limit  = 15*pi/180 ; % 负角度时更慢的变化速率 (rad/s)
    ctrl.deep_neg_phi_rate_limit = 12*pi/180 ; % 深度反转且负角度的变化速率 (rad/s)

    % 旋转抑制模式（大角度温和回正）
    ctrl.spin_theta_thresh  = 2.5 ;    % 进入旋转抑制阈值 (rad)
    ctrl.spin_theta_exit    = 2.0 ;    % 退出旋转抑制阈值 (rad)
    ctrl.spin_clf_lambda    = 1.6 ;    % 旋转抑制 CLF 斜率
    ctrl.spin_clf_k         = 2.2 ;    % 旋转抑制 CLF 收敛系数
    ctrl.spin_psi_limit     = 35*pi/180 ; % 旋转抑制时摆角限制
    ctrl.spin_phi_rate_limit = 12*pi/180 ; % 旋转抑制时期望角变化速率
    ctrl.spin_fT             = 0.2 ;   % 旋转抑制时固定小推力
    ctrl.spin_dth_exit       = 0.15 ;  % 角速度恢复阈值 (rad/s)
    ctrl.spin_dth_target     = 0.3 ;   % 目标角速度阈值 (rad/s)
    ctrl.spin_dth_cap        = 1.0 ;   % 旋转抑制时角速度限幅 (rad/s)
    ctrl.spin_ddtheta_max    = 0.8 ;   % 旋转抑制时角加速度限幅 (rad/s^2)
    ctrl.spin_tau_max        = 3.0 ;   % 旋转抑制时扭矩限幅
    ctrl.spin_recovery_hold  = 1 ;     % 旋转抑制锁存，避免抖动
    ctrl.debug_state_chain   = 1 ;     % 状态链路调试日志
    ctrl.spin_recovery_hold  = 1 ;     % 旋转抑制锁存，避免抖动
    ctrl.spin_dth_target     = 0.3 ;   % 目标角速度阈值 (rad/s)
    ctrl.debug_state_chain   = 1 ;     % 状态链路调试日志

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
