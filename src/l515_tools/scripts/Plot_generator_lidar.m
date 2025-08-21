function Plot_generator_lidar(filename)
    % Read the CSV file
    data = csvread(filename);
    
    % Extract relevant columns
    exp_x = data.roi_x;
    exp_y = data.roi_y;
    exp_z = data.roi_z;
    teo_x = data.aruco_x;
    teo_y = data.aruco_y;
    teo_z = data.aruco_z;
    dx = data.dx;
    dy = data.dy;
    dz = data.dz;
    err_norm = data.err_norm;
    
    % Create a figure for plotting
    figure;
    
    % Plot ROI vs Aruco values
    subplot(3, 1, 1);
    plot(exp_x, 'r', 'DisplayName', 'ROI X');
    hold on;
    plot(teo_x, 'b', 'DisplayName', 'Aruco X');
    title('ROI vs Aruco X');
    xlabel('Sample Index');
    ylabel('Value');
    legend;
    grid on;
    
    subplot(3, 1, 2);
    plot(exp_y, 'r', 'DisplayName', 'ROI Y');
    hold on;
    plot(teo_y, 'b', 'DisplayName', 'Aruco Y');
    title('ROI vs Aruco Y');
    xlabel('Sample Index');
    ylabel('Value');
    legend;
    grid on;
    
    subplot(3, 1, 3);
    plot(exp_z, 'r', 'DisplayName', 'ROI Z');
    hold on;
    plot(teo_z, 'b', 'DisplayName', 'Aruco Z');
    title('ROI vs Aruco Z');
    xlabel('Sample Index');
    ylabel('Value');
    legend;
    grid on;
    
    % Create a new figure for differences
    figure;
    
    % Plot differences
    subplot(3, 1, 1);
    plot(dx, 'k', 'DisplayName', 'Difference X');
    title('Difference in X');
    xlabel('Sample Index');
    ylabel('Difference');
    legend;
    grid on;
    
    subplot(3, 1, 2);
    plot(dy, 'k', 'DisplayName', 'Difference Y');
    title('Difference in Y');
    xlabel('Sample Index');
    ylabel('Difference');
    legend;
    grid on;
    
    subplot(3, 1, 3);
    plot(dz, 'k', 'DisplayName', 'Difference Z');
    title('Difference in Z');
    xlabel('Sample Index');
    ylabel('Difference');
    legend;
    grid on;
    
    % Create a new figure for normalized error
    figure;
    plot(err_norm, 'm', 'DisplayName', 'Normalized Error');
    title('Normalized Error');
    xlabel('Sample Index');
    ylabel('Error Norm');
    legend;
    grid on;
end