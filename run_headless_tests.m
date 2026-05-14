function run_headless_tests(max_cases)
%RUN_HEADLESS_TESTS Run fixed initial-condition tests headless.
%   run_headless_tests(MAX_CASES) runs up to MAX_CASES cases in order.

    if nargin < 1
        max_cases = inf;
    end
    consts = get_consts();

    % Hide figures for headless runs.
    set(0, 'DefaultFigureVisible', 'off');
    cleanup = onCleanup(@() set(0, 'DefaultFigureVisible', 'on'));

    cases = [];
    add_case = @(y0,z0,th0,dy0,dz0,dth0,wind) [y0, z0, th0, dy0, dz0, dth0, wind];

    % Deterministic coverage with altitude-dependent angle limits.
    z_list = [50, 100, 200, 350, 500, 800, 1200];
    y_list = [0, 20, -20, 50, -50, 80, -80];
    dy_list = [0, 3, -3, 6, -6, 10, -10];
    dz_list = [0, -2, -4, -6, -8, -10];
    dth_list = [0, 0.6, -0.6, 1.2, -1.2, 2.0, -2.0, 3.0, -3.0];
    wind_list = [0, 3, -3, 6, -6, 10, -10];

    for z0 = z_list
        if(z0 <= 100)
            th_list = [0, 0.15, -0.15, 0.3, -0.3, 0.5, -0.5];
        elseif(z0 <= 200)
            th_list = [0, 0.25, -0.25, 0.6, -0.6, 1.0, -1.0];
        elseif(z0 <= 500)
            th_list = [0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2];
        elseif(z0 <= 800)
            th_list = [0, 0.6, -0.6, 1.2, -1.2, 1.8, -1.8, 2.4, -2.4];
        else
            th_list = [0, 0.8, -0.8, 1.5, -1.5, 2.2, -2.2, 3.0, -3.0];
        end

        for th0 = th_list
            y0 = y_list(mod(length(cases), length(y_list)) + 1);
            dy0 = dy_list(mod(length(cases), length(dy_list)) + 1);
            dz0 = dz_list(mod(length(cases), length(dz_list)) + 1);
            dth0 = dth_list(mod(length(cases), length(dth_list)) + 1);
            wind = wind_list(mod(length(cases), length(wind_list)) + 1);
            cases = [cases; add_case(y0, z0, th0, dy0, dz0, dth0, wind)];
        end
    end

    % Add a few combined moderate cases near the middle of the range.
    cases = [cases;
             add_case(30, 200, 0.5, 5, -6, 1.0, 5);
             add_case(-30, 200, -0.5, -5, -6, -1.0, -5);
             add_case(60, 350, 0.8, 8, -8, 1.2, 8);
             add_case(-60, 350, -0.8, -8, -8, -1.2, -8)];

    num_tests = min(size(cases, 1), max_cases);
    for k = 1:num_tests
        y0 = cases(k, 1);
        z0 = cases(k, 2);
        th0 = cases(k, 3);
        dy0 = cases(k, 4);
        dz0 = cases(k, 5);
        dth0 = cases(k, 6);
        wind = cases(k, 7);

        x0 = [y0; z0; th0; 0; dy0; dz0; dth0; 0; ...
              consts.m_nofuel + consts.max.m_fuel];

        fprintf('\n=== Test %d/%d ===\n', k, num_tests);
        fprintf(['IC: y=%.2f z=%.2f th=%.3frad dy=%.2f dz=%.2f ' ...
             'dth=%.3frad/s wind=%.2f\n'], ...
            y0, z0, th0, dy0, dz0, dth0, wind);

        sim_rocket(x0, wind);
    end
end
