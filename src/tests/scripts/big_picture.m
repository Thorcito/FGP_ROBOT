figure;
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

% ---- Test 1 - first throw ----
nexttile;
t = pi_atmp1.DeltaT_ms;
yyaxis left
plot(t, pi_atmp1.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, pi_atmp1.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(342, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 1 - First Throw');
grid on;
legend('Position Error','Orientation Error','Time to intercept');

% ---- Test 1 - second throw ----
nexttile;
t = pi_atmp1_1.DeltaT_ms;
yyaxis left
plot(t, pi_atmp1_1.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, pi_atmp1_1.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(340, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 1 - Second Throw');
grid on;

% ---- Test 2 - first throw ----
nexttile;
t = pi_atmp2.DeltaT_ms;
yyaxis left
plot(t, pi_atmp2.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, pi_atmp2.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(387, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 2 - First Throw');
grid on;

% ---- Test 2 - second throw ----
nexttile;
t = pi_atmp2_2.DeltaT_ms;
yyaxis left
plot(t, pi_atmp2_2.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, pi_atmp2_2.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(390, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 2 - Second Throw');
grid on;

% ---- Test 3 - first throw ----
nexttile;
t = pi_atmp3.DeltaT_ms;
yyaxis left
plot(t, pi_atmp3.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, pi_atmp3.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(335, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 3 - First Throw');
grid on;

% ---- Test 3 - second throw ----
nexttile;
t = pi_atmp3_3.DeltaT_ms;
yyaxis left
plot(t, pi_atmp3_3.Error_mm, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
yyaxis right
plot(t, pi_atmp3_3.Angle_deg, 'r', 'LineWidth', 1.5);
ylabel('Orientation Error (°)');
xline(387, '--k', 'Intercept (t)');
xlabel('Time (ms)');
title('Attempt 3 - Second Throw');
grid on;

