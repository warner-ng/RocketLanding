function sweep_high_rate_scale()
    vals = [0.55 0.65 0.70 0.80];
    for v = vals
        patch_file_value('student_setup.m', 'ctrl.high_rate_vz_scale = ', v);
        fprintf('\n=== high_rate_vz_scale = %.2f ===\n', v);
        R = batch_random_range_test(30,2);
        fprintf('count=%d mean=%.2f\n', sum(R(:,1)>0), mean(R(:,1)));
    end
end

function patch_file_value(fname, prefix, val)
    txt = fileread(fname);
    expr = [regexptranslate('escape', prefix) '[0-9.]+ ;'];
    repl = sprintf('%s%.2f ;', prefix, val);
    txt = regexprep(txt, expr, repl);
    fid = fopen(fname,'w'); fwrite(fid,txt); fclose(fid);
end
