% Generate specific figures for ballistic analysis
function ballistic_specific_figures()
    % File names
    filenames = {
        '2d_grav_1.csv', 
        '2d_grav_2.csv', 
        '2d_grav_3.csv', 
        '2d_grav_4.csv', 
        '2d_grav_5.csv'
    };
    
    % Colors for different datasets
    colors = ['b', 'r', 'g', 'm', 'c'];
    markers = ['o', 's', '^', 'd', 'v'];
    
    %% Figure 1: All u positions (measured and fitted)
    figure('Position', [100, 100, 1200, 800]);
    hold on;
    
    for i = 1:5
        if exist(filenames{i}, 'file')
            data = readtable(filenames{i});
            
            % Plot measured u positions
            plot(data.t_s, data.u_meas, [colors(i) markers(i)], ...
                 'MarkerSize', 5, 'MarkerFaceColor', colors(i), ...
                 'DisplayName', ['Run ' num2str(i) ' Measured']);
            
            % Plot fitted u positions
            plot(data.t_s, data.u_fit, [colors(i) '-'], ...
                 'LineWidth', 2, 'DisplayName', ['Run ' num2str(i) ' Fitted']);
        else
            fprintf('File %s not found\n', filenames{i});
        end
    end
    
    xlabel('Time (s)');
    ylabel('u position (pixels)');
    title('All U Positions - Measured vs Fitted');
    legend('Location', 'best', 'NumColumns', 2);
    grid on;
    set(gca, 'FontSize', 12);
    
    %% Figure 2: All v positions (measured and fitted)
    figure('Position', [100, 100, 1200, 800]);
    hold on;
    
    for i = 1:5
        if exist(filenames{i}, 'file')
            data = readtable(filenames{i});
            
            % Plot measured v positions
            plot(data.t_s, data.v_meas, [colors(i) markers(i)], ...
                 'MarkerSize', 5, 'MarkerFaceColor', colors(i), ...
                 'DisplayName', ['Run ' num2str(i) ' Measured']);
            
            % Plot fitted v positions
            plot(data.t_s, data.v_fit, [colors(i) '-'], ...
                 'LineWidth', 2, 'DisplayName', ['Run ' num2str(i) ' Fitted']);
        end
    end
    
    xlabel('Time (s)');
    ylabel('v position (pixels)');
    title('All V Positions - Measured vs Fitted');
    legend('Location', 'best', 'NumColumns', 2);
    grid on;
    set(gca, 'FontSize', 12);
    
    %% Figure 3: All trajectories (measured and fitted)
    figure('Position', [100, 100, 1200, 800]);
    hold on;
    
    for i = 1:5
        if exist(filenames{i}, 'file')
            data = readtable(filenames{i});
            
            % Plot measured trajectories
            plot(data.v_meas, data.u_meas, [colors(i) markers(i)], ...
                 'MarkerSize', 5, 'MarkerFaceColor', colors(i), ...
                 'DisplayName', ['Run ' num2str(i) ' Measured']);
            
            % Plot fitted trajectories
            plot(data.v_fit, data.u_fit, [colors(i) '-'], ...
                 'LineWidth', 2, 'DisplayName', ['Run ' num2str(i) ' Fitted']);
        end
    end
    
    xlabel('u position (pixels)');
    ylabel('v position (pixels)');
    title('All Trajectories - Measured vs Fitted');
    legend('Location', 'best', 'NumColumns', 2);
    grid on;
    axis equal;
    set(gca, 'FontSize', 12);
    
    fprintf('Generated 3 figures:\n');
    fprintf('1. All U positions (measured and fitted)\n');
    fprintf('2. All V positions (measured and fitted)\n');
    fprintf('3. All trajectories (measured and fitted)\n');
    
end