% Reproduce teacher's LQR K
% State: x = [y, z, th, psi, dy, dz, dth, dpsi, m]
% Input: u = [fT, tau]
function K = reproduce_teacher_LQR()
    consts = get_consts();

    g   = consts.g;       % 9.81
    gam = consts.gamma;   % 1000
    L   = consts.L;       % 10
    Jm  = consts.Jm;      % L^2/3
    JT  = consts.JT;
    m0  = consts.m_nofuel + consts.max.m_fuel;  % 175 kg (full fuel)

    % State: x = [y, z, th, psi, dy, dz, dth, dpsi, m]
    % ---- Linearize at x_eq=[0,L,0,0,0,0,0,0,m0], u_eq=[m0*g/gam, 0] ----
    % Derivation of nonzero A entries:
    %   dy/dt   = dy                       -> A(1,5)=1
    %   dz/dt   = dz                       -> A(2,6)=1
    %   dth/dt  = dth                      -> A(3,7)=1
    %   dpsi/dt = dpsi                     -> A(4,8)=1
    %   d(dy)/dt  = -gam*sin(th+psi)/m*fT
    %     d/dth|eq = -gam*cos(0)/m0*(m0*g/gam) = -g  -> A(5,3)=-g
    %     d/dpsi   = same                             -> A(5,4)=-g
    %   d(dz)/dt  = gam*cos(th+psi)/m*fT - g
    %     d/dm|eq  = -gam*cos(0)/m0^2*(m0*g/gam) = -g/m0 -> A(6,9)=-g/m0
    %   d(dth)/dt = -L*gam*sin(psi)/(Jm*m)*fT
    %     d/dpsi|eq= -L*gam*cos(0)/(Jm*m0)*(m0*g/gam) = -L*g/Jm -> A(7,4)=-L*g/Jm


    %        y  z  θ   ψ    ẏ  ż  θ̇  ψ̇    m
    % A = [  0  0  0   0    1  0  0  0    0  ]   ẏ
    %     [  0  0  0   0    0  1  0  0    0  ]   ż
    %     [  0  0  0   0    0  0  1  0    0  ]   θ̇
    %     [  0  0  0   0    0  0  0  1    0  ]   ψ̇
    %     [  0  0 -g  -g    0  0  0  0    0  ]   ÿ    ← tilt + gimbal → horizontal accel
    %     [  0  0  0   0    0  0  0  0  -g/m₀]   z̈   ← heavier = less accel per fT
    %     [  0  0  0 -Lg/Jm 0  0  0  0    0  ]   θ̈   ← gimbal torques body
    %     [  0  0  0   0    0  0  0  0    0  ]   ψ̈
    %     [  0  0  0   0    0  0  0  0    0  ]   ṁ

    %        fT       tau
    % B = [  0         0   ]
    %     [  0         0   ]
    %     [  0         0   ]
    %     [  0         0   ]
    %     [  0         0   ]   ← sin(0)=0: thrust can't push sideways at ψ=0
    %     [ γ/m₀       0   ]   ← thrust drives vertical accel
    %     [  0         0   ]   ← sin(0)=0: thrust can't torque body at ψ=0
    %     [  0        1/JT ]   ← torque drives gimbal accel
    %     [ -1         0   ]   ← thrust burns fuel

    % most crucial part of the LQR calculation
    A = zeros(9);
    A(1,5) = 1;
    A(2,6) = 1;
    A(3,7) = 1;
    A(4,8) = 1;
    A(5,3) = -g;
    A(5,4) = -g;
    A(6,9) = -g / m0;
    A(7,4) = -L*g / Jm;

    % g_vec evaluated at eq (th=0, psi=0):
    %   B(6,1) = gam*cos(0)/m0 = gam/m0
    %   B(8,2) = 1/JT
    %   B(9,1) = -1
    B = zeros(9,2);
    B(6,1) = gam / m0;
    B(8,2) = 1 / JT;
    B(9,1) = -1;

    % ---- Q and R (Bryson's rule from problem max values) ----
    % Q denotes state cost, R denotes input cost.  LQR finds K to minimize x'*Q*x + u'*R*u.
    Q = diag([1/20^2,         % y
              1/L^2,           % z   (L=10m)
              1/(pi/6)^2,      % th
              1/(pi/6)^2,      % psi
              1/5^2,           % dy
              1/5^2,           % dz
              1/1^2,           % dth
              1/1^2,           % dpsi
              0]);             % m   (feedforward handles gravity)

    R = diag([1/consts.max.fT^2,   % fT  (max 25 kg/s)
              1/consts.max.tau^2]); % tau (max 10 N*m)

    K = lqr(A, B, Q, R);

    fprintf('\n--- Your computed K ---\n');
    disp(K)
    fprintf('--- Teacher K ---\n');
    disp([0.0000  0.0018  0.0997  0.4690 -0.0005  0.0353  0.4850  1.8646  0.0000;
          0.0000  0.0000 -0.0036  1.0045  0.0000  0.0000 -0.0469  3.8980  0.0000])
end
