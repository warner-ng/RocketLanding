function run_headless_tests(max_cases, theta_step_rad)
%RUN_HEADLESS_TESTS Run fixed initial-condition tests headless.
%   run_headless_tests(MAX_CASES, THETA_STEP_RAD) runs up to MAX_CASES cases.

    if nargin < 1
        max_cases = inf;
    end
    if nargin < 2
        theta_step_rad = 0.1;
    end
    consts = get_consts();

    % Hide figures for headless runs.
    set(0, 'DefaultFigureVisible', 'off');
    cleanup = onCleanup(@() set(0, 'DefaultFigureVisible', 'on'));
    opts.verbose = true;
    opts.do_plots = false;
    opts.do_anim = false;
    use_ansi = true;
    save_summary_plot = true;

    cases = [];
    add_case = @(y0,z0,th0,dy0,dz0,dth0,wind) [y0, z0, th0, dy0, dz0, dth0, wind];

    % Deterministic coverage: vary only altitude and attitude.
    % Small angles at high altitude are known-passing; skip them to save time.
    % Focus on problem regions: large angles at high altitude.
    z_list = [25, 50, 100, 200, 350, 500, 800, 1200, 1500];
    th_grid = -3:theta_step_rad:3;

    for z0 = z_list
        if(z0 <= 100)
            max_th = 0.5;
        elseif(z0 <= 200)
            max_th = 1.0;
        elseif(z0 <= 500)
            max_th = 1.2;
        elseif(z0 <= 800)
            max_th = 2.4;
        elseif(z0 <= 1200)
            % z=1200: small angles (|th|<=2.0) pass reliably, only test edges
            th_list_z = th_grid(abs(th_grid) <= 3.0 & ...
                                (abs(th_grid) >= 1.8 | abs(th_grid) <= 0.1));
            for th0 = th_list_z
                cases = [cases; add_case(0, z0, th0, 0, 0, 0, 0)];
            end
            continue;
        else
            % z=1500: small angles (|th|<=1.8) pass reliably, only test edges
            th_list_z = th_grid(abs(th_grid) <= 3.0 & ...
                                (abs(th_grid) >= 1.8 | abs(th_grid) <= 0.1));
            for th0 = th_list_z
                cases = [cases; add_case(0, z0, th0, 0, 0, 0, 0)];
            end
            continue;
        end
        th_list = th_grid(abs(th_grid) <= max_th);

        for th0 = th_list
            cases = [cases; add_case(0, z0, th0, 0, 0, 0, 0)];
        end
    end

    num_tests = min(size(cases, 1), max_cases);
    scores = nan(num_tests, 1);
    zs = cases(1:num_tests, 2);
    ths = cases(1:num_tests, 3);
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

        [~, ~, J] = sim_rocket(x0, wind, opts);
        scores(k) = J;
    end

    % Summary table (rows: height, cols: angle)
    unique_z = z_list;
    unique_th = th_grid;
    pass = scores > 0;

    red = '';
    reset = '';
    if(use_ansi)
        red = sprintf('\x1b[31m');
        reset = sprintf('\x1b[0m');
    end

    if(numel(unique_th) <= 40)
        fprintf('\n=== Summary (OK in red) ===\n');
        fprintf('z\\th |');
        for j = 1:numel(unique_th)
            fprintf(' %6.2f ', unique_th(j));
        end
        fprintf('\n');

        for i = 1:numel(unique_z)
            fprintf('%4.0f |', unique_z(i));
            for j = 1:numel(unique_th)
                idx = find(zs == unique_z(i) & abs(ths - unique_th(j)) < 1e-9, 1, 'first');
                if(isempty(idx))
                    cell_txt = '   N/A';
                else
                    if(pass(idx))
                        cell_txt = sprintf('%s  OK %s', red, reset);
                    else
                        cell_txt = ' FAIL ';
                    end
                end
                fprintf(' %6s', cell_txt);
            end
            fprintf('\n');
        end
    else
        fprintf('\n=== Summary table skipped (too many angle bins) ===\n');
    end

    % CSV export for analysis (timestamped)
    summary = table(zs, ths, scores, pass, ...
                    'VariableNames', {'z','th','score','pass'});
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    out_dir = fullfile(fileparts(mfilename('fullpath')), 'outputs');
    csv_name = fullfile(out_dir, ['headless_summary_' stamp '.csv']);
    writetable(summary, csv_name);

    % Visual summary (heatmap): green = pass, red = fail, gray = N/A
    if(save_summary_plot)
        pass_grid = nan(numel(unique_z), numel(unique_th));
        for i = 1:numel(unique_z)
            for j = 1:numel(unique_th)
                idx = find(zs == unique_z(i) & abs(ths - unique_th(j)) < 1e-9, 1, 'first');
                if(~isempty(idx))
                    pass_grid(i, j) = double(pass(idx));
                end
            end
        end

        fig = figure('Visible', 'off');
        plot_grid = pass_grid;
        plot_grid(isnan(plot_grid)) = 0.5;
        imagesc(unique_th, unique_z, plot_grid);
        colormap([0.8 0.2 0.2; 0.7 0.7 0.7; 0.2 0.7 0.2]);
        caxis([0 1]);
        colorbar('Ticks', [0 0.5 1], 'TickLabels', {'FAIL', 'N/A', 'OK'});
        xlabel('theta (rad)');
        ylabel('z (m)');
        title('Pass/Fail Heatmap');
        set(gca, 'YDir', 'normal');
        xticks(-3:0.5:3);
        png_name = fullfile(out_dir, ['headless_summary_' stamp '.png']);
        saveas(fig, png_name);
        close(fig);
    end
end
