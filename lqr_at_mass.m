% Compute LQR gain K linearized at rocket mass m.
% A(6,9)=-g/m and B(6,1)=gam/m are the only mass-dependent entries.
function K = lqr_at_mass(m, consts)
    g   = consts.g ;
    gam = consts.gamma ;
    L   = consts.L ;
    Jm  = consts.Jm ;
    JT  = consts.JT ;

    A = zeros(9) ;
    A(1,5) = 1 ;       A(2,6) = 1 ;
    A(3,7) = 1 ;       A(4,8) = 1 ;
    A(5,3) = -g ;      A(5,4) = -g ;
    A(6,9) = -g/m ;
    A(7,4) = -L*g/Jm ;

    B = zeros(9,2) ;
    B(6,1) = gam/m ;
    B(8,2) = 1/JT ;
    B(9,1) = -1 ;

    Q = diag([1/20^2, 1/L^2, 1/(pi/6)^2, 1/(pi/6)^2, ...
              1/5^2,  1/5^2, 1/1^2,       1/1^2,  0]) ;
    R = diag([1/consts.max.fT^2, 1/consts.max.tau^2]) ;

    K = lqr(A, B, Q, R) ;
end
