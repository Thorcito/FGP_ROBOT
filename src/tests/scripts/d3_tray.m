% Simple 3D trajectory plot
data = readtable('std_motion.csv'); % Replace with your actual filename

figure;
plot3(data.x, data.y, data.z, 'b-.', 'LineWidth', 2);
hold on;
scatter3(data.x(1), data.y(1), data.z(1), 100, 'g', 'filled', 'MarkerEdgeColor', 'k');
scatter3(data.x(end), data.y(end), data.z(end), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory');
legend('Trajectory', 'Start', 'End', 'Location', 'best');
axis equal;

% Add color coding based on time or standard deviation
% Uncomment one of the following lines for color coding:

% Color by time:
% scatter3(data.x, data.y, data.z, 20, data.dt_s, 'filled');
% colorbar; title('Trajectory colored by time');

% Color by 3D STD:
% scatter3(data.x, data.y, data.z, 20, data.std_3d, 'filled');
% colorbar; title('Trajectory colored by 3D STD');