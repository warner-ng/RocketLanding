consts = get_consts();

% Random IC within grading range
y0   = (2*rand-1) * 100;                    % [-100, 100] m
z0   = 25 + rand * (1500-25);               % [25, 1500] m
th0  = (2*rand-1) * 179 * pi/180;          % [-179, 179] deg
dy0  = (2*rand-1) * 10;                     % [-10, 10] m/s
dz0  = (2*rand-1) * 10;                     % [-10, 10] m/s
dth0 = (2*rand-1) * 179 * pi/180;          % [-179, 179] deg/s
wind = (2*rand-1) * 10;                     % [-10, 10] N

x0 = [y0; z0; th0; 0; dy0; dz0; dth0; 0; consts.m_nofuel + consts.max.m_fuel];
sim_rocket(x0, wind);