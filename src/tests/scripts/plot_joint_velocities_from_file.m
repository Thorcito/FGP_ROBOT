function plot_joint_velocities_multi(input_arg)
% Plot joint velocities for multiple experiments (text files with lines like:
% [INFO] ... VELOCITAAAT:  v1 v2 v3 v4 v5 v6 )
%
% Usage:
%   plot_joint_velocities_multi('folder_with_txts')
%   plot_joint_velocities_from_file({'velocities.txt','velocities_2.txt', 'velocities_3.txt', 'velocities_4.txt', 'velocities_5.txt', 'velocities_6.txt', 'velocities_7.txt'})

    % -------- Settings --------
    sample_dt = 0.002;         % seconds per sample
    pattern   = 'VELOCITAAAT:'; % token to search in each line
    save_png  = false;          % save figure as PNG
    save_csv  = true;          % save stats as CSV
    csv_name  = 'joint_velocity_stats.csv';

    % -------- Resolve file list --------
    if ischar(input_arg) || (isstring(input_arg) && isscalar(input_arg))
        folder = char(input_arg);
        files = dir(fullfile(folder, '*.txt'));
        filenames = fullfile({files.folder}, {files.name});
        if isempty(filenames)
            error('No .txt files found in folder: %s', folder);
        end
    elseif iscell(input_arg)
        filenames = input_arg;
    else
        error('Input must be a folder path or a cell array of filenames.');
    end

    % -------- Read all experiments --------
    nExp = numel(filenames);
    expData = cell(nExp,1); % each cell: Nx6 matrix
    expNames = strings(nExp,1);

    for e = 1:nExp
        [q_vel, line_count] = read_one_file(filenames{e}, pattern);
        if isempty(q_vel)
            warning('No valid data in %s (scanned %d lines). Skipping.', filenames{e}, line_count);
            continue;
        end
        expData{e} = q_vel;
        [~,base,~] = fileparts(filenames{e});
        expNames(e) = string(base);
        fprintf('Loaded %s : %d samples (from %d lines)\n', base, size(q_vel,1), line_count);
    end

    % remove empties (if any)
    keep = ~cellfun(@isempty, expData);
    expData = expData(keep);
    expNames = expNames(keep);
    nExp = numel(expData);
    if nExp == 0
        error('No experiments contained valid data.');
    end

    % -------- Plot: one subplot per joint, curves = experiments --------
    J = 6; % joints
    f = figure('Position',[100 100 1200 800],'Name','Joint Velocities Across Experiments');
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    for j = 1:J
        nexttile;
        hold on; grid on;
        for e = 1:nExp
            qj = expData{e}(:,j);
            t  = (0:numel(qj)-1) * sample_dt;
            plot(t, qj, 'LineWidth', 1.2, 'DisplayName', expNames(e));
        end
        title(sprintf('Joint %d Velocity', j));
        xlabel('Time (s)'); ylabel('Velocity (rad/s)');
        if j == 1
            legend('Test 1', 'Test 2', 'Test 3', 'Test 4', 'Test 5', 'Test 6', 'Test 7');
        end
        hold off;
    end

    % -------- Print stats & build CSV table --------
    fprintf('\n==== Joint Velocity Statistics by Experiment ====\n');
    Stats = []; % will collect rows for CSV
    for e = 1:nExp
        q = expData{e};
        fprintf('\nExperiment: %s\n', expNames(e));
        for j = 1:J
            m  = mean(q(:,j));
            sd = std(q(:,j));
            mx = max(q(:,j));
            mn = min(q(:,j));
            fprintf('  Joint %d: Mean=%8.6f  Std=%8.6f  Max=%8.6f  Min=%8.6f\n', j, m, sd, mx, mn);
            Stats = [Stats; {char(expNames(e)), j, m, sd, mx, mn}]; %#ok<AGROW>
        end
    end

    % -------- Save CSV (optional) --------
    if save_csv
        T = cell2table(Stats, 'VariableNames', ...
            {'Experiment','Joint','Mean','Std','Max','Min'});
        try
            writetable(T, csv_name);
            fprintf('\nSaved stats CSV: %s\n', csv_name);
        catch ME
            warning('Could not save CSV (%s): %s', csv_name, ME.message);
        end
    end

    % -------- Save PNG (optional) --------
    if save_png
        out_png = sprintf('joint_velocities_%s.png', datestr(now,'yyyymmdd_HHMMSS'));
        try
            exportgraphics(f, out_png, 'Resolution', 200);
            fprintf('Saved figure PNG: %s\n', out_png);
        catch ME
            warning('Could not save PNG: %s', ME.message);
        end
    end
end

% ===== Helper: read a single file into Nx6 double =====
function [q_vel, line_count] = read_one_file(filename, token)
    fid = fopen(filename,'r');
    if fid == -1
        error('Could not open file: %s', filename);
    end
    q_vel = [];
    line_count = 0;

    while ~feof(fid)
        line = fgetl(fid);
        line_count = line_count + 1;

        if ~ischar(line) || isempty(line)
            continue;
        end

        if contains(line, token)
            % grab everything after the token
            idx = strfind(line, token);
            numeric_part = strtrim(line(idx + length(token): end));

            % robust parse: read any amount of floats on the line
            nums = sscanf(numeric_part, '%f')'; % row vector
            if numel(nums) == 6
                q_vel = [q_vel; nums]; %#ok<AGROW>
            else
                % Try regex fallback if trailing text present
                tk = regexp(line, 'VELOCITAAAT:\s+([-\d\.\sEe\+]+)', 'tokens');
                if ~isempty(tk)
                    nums2 = sscanf(tk{1}{1}, '%f')';
                    if numel(nums2) == 6
                        q_vel = [q_vel; nums2]; %#ok<AGROW>
                    else
                        fprintf('Warning %s (line %d): got %d numbers (expected 6)\n', filename, line_count, numel(nums2));
                    end
                else
                    fprintf('Warning %s (line %d): token found but numbers not parsed.\n', filename, line_count);
                end
            end
        end
    end
    fclose(fid);
end
