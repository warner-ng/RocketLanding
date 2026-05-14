function analyze_failures(csv_path)
    % Analyze failure reasons for a headless summary CSV.
    if(nargin < 1 || isempty(csv_path))
        files = dir(fullfile('outputs', 'headless_summary_*.csv'));
        if(isempty(files))
            error('No headless summary CSV found.');
        end
        [~, idx] = max([files.datenum]);
        csv_path = fullfile(files(idx).folder, files(idx).name);
    end

    consts = get_consts();
    T = readtable(csv_path);
    fail_idx = find(T.pass == 0);
    if(isempty(fail_idx))
        fprintf('No failures in %s\n', files(idx).name);
        return;
    end

    opts.verbose = false;
    opts.do_plots = false;
    opts.do_anim = false;

    reason_labels = {'y', 'z', 'theta', 'speed', 'dtheta', 'time'};
    counts_all = zeros(1, numel(reason_labels));
    counts_hi = zeros(1, numel(reason_labels));

    hi_filter = abs(T.th) >= 1.8 & T.z >= 500;

    for k = 1:numel(fail_idx)
        i = fail_idx(k);
        z0 = T.z(i);
        th0 = T.th(i);
        x0 = [0; z0; th0; 0; 0; 0; 0; 0; consts.m_nofuel + consts.max.m_fuel];
        [t, x] = sim_rocket(x0, 0, opts);
        xe = x(end, :)';

        y = xe(1);
        z = xe(2);
        th = xe(3);
        dy = xe(5);
        dz = xe(6);
        dth = xe(7);

        p = [y; z - consts.L; acos(cos(th)); norm([dy; dz]); dth];
        maxv = [consts.max.y; consts.max.z; consts.max.theta; consts.max.speed; consts.max.dtheta];
        viol = abs(p) > maxv;

        time_fail = t(end) >= 59.9;

        reasons = [viol(:); time_fail];
        counts_all = counts_all + reasons';

        if(hi_filter(i))
            counts_hi = counts_hi + reasons';
        end
    end

    [~, csv_name, csv_ext] = fileparts(csv_path);
    fprintf('CSV: %s%s\n', csv_name, csv_ext);
    fprintf('Failures: %d (hi-angle subset: %d)\n', numel(fail_idx), sum(hi_filter & T.pass == 0));

    fprintf('\nFailure reasons (all failures):\n');
    for j = 1:numel(reason_labels)
        fprintf('  %s: %d\n', reason_labels{j}, counts_all(j));
    end

    fprintf('\nFailure reasons (z>=500 & |th|>=1.8):\n');
    for j = 1:numel(reason_labels)
        fprintf('  %s: %d\n', reason_labels{j}, counts_hi(j));
    end
end
