figure;
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

% ---- Test 1 - first throw ----
nexttile;
t = min_atmp1.DeltaT_ms;
yyaxis left
plot(t, min_atmp1.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, min_atmp1.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(179, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 1 - Time to intercept: 179ms');
grid on;
legend('Position Error','Orientation Error','Time to intercept');

% ---- Test 2 - second throw ----
nexttile;
t = min_atmp2.DeltaT_ms;
yyaxis left
plot(t, min_atmp2.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, min_atmp2.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(193, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 2 - Time to intercept: 193ms');
grid on;

% ---- Test 2 - first throw ----
nexttile;
t = min_atmp3.DeltaT_ms;
yyaxis left
plot(t, min_atmp3.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, min_atmp3.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(177, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 3 - Time to intercept: 177ms');
grid on;

% ---- Test 2 - second throw ----
nexttile;
t = min_atmp4.DeltaT_ms;
yyaxis left
plot(t, min_atmp4.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, min_atmp4.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(165, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 4 - Time to intercept: 165ms');
grid on;

% ---- Test 3 - first throw ----
nexttile;
t = min_atmp5.DeltaT_ms;
yyaxis left
plot(t, min_atmp5.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, min_atmp5.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(140, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 5 - Time to intercept: 140ms');
grid on;

% ---- Test 3 - second throw ----
nexttile;
t = min_atmp6.DeltaT_ms;
yyaxis left
plot(t, min_atmp6.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, min_atmp6.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(132, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 6 - Time to intercept: 132ms');
grid on;

