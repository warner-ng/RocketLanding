function results = batch_old_lqr_range_test(N, seed)
    if nargin < 1, N = 30; end
    if nargin < 2, seed = 2; end
    close all; rng(seed);
    consts = get_consts();
    K = [ -0.0000 2.5000 0.0000 -0.0000 -0.0000 5.0867 0.0000 0.0000 -0.0098;
           0.5000 0.0000 -71.4795 120.1597 3.3595 -0.0000 -81.6727 40.1654 -0.0000];
    results = zeros(N, 16);
    for k=1:N
        y0=(2*rand-1)*100; z0=25+rand*(1500-25); th0=(2*rand-1)*179*pi/180;
        dy0=(2*rand-1)*10; dz0=(2*rand-1)*10; dth0=(2*rand-1)*179*pi/180; wind=(2*rand-1)*10;
        x0=[y0;z0;th0;0;dy0;dz0;dth0;0;consts.m_nofuel+consts.max.m_fuel];
        odeopts=odeset('Events',@odeevents_touchdown_local,'RelTol',1e-5,'AbsTol',1e-7);
        [t,x]=ode45(@odefun_lqr,[0 60],x0,odeopts,wind,consts,K);
        J=compute_score(x(end,:)',consts);
        results(k,:)=[J,t(end),y0,z0,th0,dy0,dz0,dth0,wind,x(end,1),x(end,2),x(end,3),norm(x(end,5:6)),x(end,7),x(end,9),size(x,1)];
        fprintf('%02d J=%6.2f t=%5.1f init[y=%7.1f z=%7.1f th=%6.1f dy=%5.1f dz=%5.1f dth=%6.1f w=%5.1f] land[y=%8.1f z=%6.1f th=%7.1f sp=%6.1f dth=%7.1f]\n',k,J,t(end),y0,z0,th0*180/pi,dy0,dz0,dth0*180/pi,wind,x(end,1),x(end,2),x(end,3)*180/pi,norm(x(end,5:6)),x(end,7)*180/pi);
    end
    fprintf('success count J>0: %d/%d, mean J: %.2f\n',sum(results(:,1)>0),N,mean(results(:,1)));
end

function [dx,u]=odefun_lqr(t,x,wind,consts,K)
    y=x(1);z=x(2);th=x(3);psi=x(4);dy=x(5);dz=x(6);dth=x(7);dpsi=x(8);m=x(9); consts.J=consts.Jm*m;
    xd=[0;consts.L;0;0;0;0;0;0;m]; uff=[m*consts.g/consts.gamma;0]; u=uff-K*(x-xd);
    if m<=consts.m_nofuel,u(1)=0;end; u(1)=min(max(u(1),consts.min.fT),consts.max.fT); u(2)=min(max(u(2),-consts.max.tau),consts.max.tau);
    f=[dy;dz;dth;dpsi;0;-consts.g;0;0;0];
    g=[0,0;0,0;0,0;0,0;-consts.gamma*sin(psi+th)/m,0;consts.gamma*cos(psi+th)/m,0;-consts.L*consts.gamma*sin(psi)/consts.J,0;0,1/consts.JT;-1,0];
    if z>2*consts.L,d=wind/m*exp(-1/(z-2*consts.L));else,d=0;end
    dx=f+g*u+[zeros(4,1);d;zeros(4,1)];
end
function [value,isterminal,direction]=odeevents_touchdown_local(t,x,wind,consts,K)
    z=x(2);th=x(3);L=consts.L;r=consts.r; value=min([z-L*cos(th)-r*sin(th);z-L*cos(th)+r*sin(th);z+L*cos(th);z-r*sin(th);z+r*sin(th)]); isterminal=1; direction=0;
end
