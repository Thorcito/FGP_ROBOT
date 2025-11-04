%% parse_tiro_log.m
% Extract position, angle, and Δt errors from ROS log lines

clc; clear; close all;

% ---------- CONFIG ----------
logFile = 'min_2_tiro2.txt';       % input log file
outCsv  = 'min_atmp6.csv';  % output CSV file
% ----------------------------

% Open file
fid = fopen(logFile, 'r');
if fid == -1
    error('No se pudo abrir el archivo: %s', logFile);
end

% Initialize arrays
pos_err = [];
ang_err = [];
dt_err  = [];

% Read file line by line
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end

    % Look for "ERROR" in the line
    if contains(line, 'ERROR:')
        % Extract numeric values between keywords
        posToken = extractBetween(line, 'ERROR:', 'm');
        angToken = extractBetween(line, 'ANGLE:', 'rad');
        dtToken  = extractBetween(line, 'Δt:', 's');

        % Convert strings to numbers
        posVal = str2double(strtrim(posToken));
        angVal = str2double(strtrim(angToken));
        dtVal  = str2double(strtrim(dtToken));

        % Store if valid numbers
        if ~isnan(posVal) && ~isnan(angVal) && ~isnan(dtVal)
            pos_err(end+1) = posVal;
            ang_err(end+1) = angVal;
            dt_err(end+1)  = dtVal;
        end
    end
end
fclose(fid);

% Check if data was found
if isempty(pos_err)
    error('No se encontraron líneas con "ERROR:" en el archivo.');
end

% Create table with results
dt_err = dt_err-dt_err(1);
T = table((1:numel(pos_err))', pos_err', ang_err', dt_err', ...
    'VariableNames', {'Attempt','Error_m','Angle_rad','DeltaT_s'});

% Add conversions to mm, deg, and ms
T.Error_mm   = T.Error_m * 1000;
T.Angle_deg  = T.Angle_rad * (180/pi);
T.DeltaT_ms  = T.DeltaT_s * 1000;

% Create time vector (assuming uniform sampling)
t = T.DeltaT_ms; % or use your actual timestamps if available

figure;
yyaxis left
plot(t, T.Error_mm, 'b', 'LineStyle', '--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, T.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');


%yline(15, '--k', 'Threshold (°)');
xline(133, '--k', 'Threshold (t)');

xlabel('Time to Interception (ms)');
title('End-Effector Error Convergence During Interception');
grid on;
legend('Position Error','Orientation Error','Threshold');

% Save to CSV
writetable(T, outCsv);fprintf('✅ Se extrajeron %d intentos.\n', height(T));fprintf('📄 Archivo CSV guardado en: %s\n', outCsv);

% Optional: show summary
fprintf('\nResumen:\n');
fprintf('  Error promedio posición: %.2f mm\n', mean(T.Error_mm));
fprintf('  Error promedio orientación: %.3f°\n', mean(T.Angle_deg));
fprintf('  Δt promedio: %.2f ms\n', mean(T.DeltaT_ms));
