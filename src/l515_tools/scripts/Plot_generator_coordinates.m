%Funcion para extraer y plotear los datos de coordenadas calculados por la
%camara en comparacion con los datos obtenidos de un marcador aruco

function Plot_generator_coordinates()
    % Read the CSV file
    data = readmatrix("aruco_roi_results.csv");
    disp(data)

    num_experiments = 5; % Number of experiments
    rows_per_experiment = 10; % Number of rows per experiment

    % Initialize arrays to hold all data for 3D plot
    all_exp_x = [];
    all_exp_y = [];
    all_exp_z = [];
    all_teo_x = [];
    all_teo_y = [];
    all_teo_z = [];
    
    for i = 1:num_experiments
        % Extract data for the current experiment
        start_row = (i-1) * rows_per_experiment + 1;
        end_row = start_row + rows_per_experiment - 1;
        
        exp_x = data(start_row:end_row, 5);
        exp_y = data(start_row:end_row, 6);
        exp_z = data(start_row:end_row, 7);
        teo_x = data(start_row:end_row, 8);
        teo_y = data(start_row:end_row, 9);
        teo_z = data(start_row:end_row, 10);
        dx = data(start_row:end_row, 11);
        dy = data(start_row:end_row, 12);
        dz = data(start_row:end_row, 13);
        err_norm = data(start_row:end_row, 14);
        
        % Create a figure for plotting
        figure;
        
        % Plot ROI vs Aruco values
        subplot(3, 2, 1);
        plot(exp_x, 'r', 'DisplayName', 'ROI X');
        hold on;
        plot(teo_x, 'b', 'DisplayName', 'Aruco X', 'LineStyle','--');
        title(['Experiment ' num2str(i) ': ROI vs Aruco X']);
        xlabel('Sample Index');
        ylabel('Value');
        legend;
        grid on;
        
        subplot(3, 2, 2);
        plot(dx, 'k', 'DisplayName', 'Difference X');
        title(['Experiment ' num2str(i) ': Difference in X']);
        xlabel('Sample Index');
        ylabel('Difference');
        legend;
        grid on;
        
        subplot(3, 2, 3);
        plot(exp_y, 'r', 'DisplayName', 'ROI Y');
        hold on;
        plot(teo_y, 'b', 'DisplayName', 'Aruco Y', 'LineStyle','--');
        title(['Experiment ' num2str(i) ': ROI vs Aruco Y']);
        xlabel('Sample Index');
        ylabel('Value');
        legend;
        grid on;
        
        subplot(3, 2, 4);
        plot(dy, 'k', 'DisplayName', 'Difference Y');
        title(['Experiment ' num2str(i) ': Difference in Y']);
        xlabel('Sample Index');
        ylabel('Difference');
        legend;
        grid on;
        
        subplot(3, 2, 5);
        plot(exp_z, 'r', 'DisplayName', 'ROI Z');
        hold on;
        plot(teo_z, 'b', 'DisplayName', 'Aruco Z', 'LineStyle','--');
        title(['Experiment ' num2str(i) ': ROI vs Aruco Z']);
        xlabel('Sample Index');
        ylabel('Value');
        legend;
        grid on;
        
        subplot(3, 2, 6);
        plot(dz, 'k', 'DisplayName', 'Difference Z');
        title(['Experiment ' num2str(i) ': Difference in Z']);
        xlabel('Sample Index');
        ylabel('Difference');
        legend;
        grid on;
        
        % Store data for 3D plot
        all_exp_x = [all_exp_x; exp_x];
        all_exp_y = [all_exp_y; exp_y];
        all_exp_z = [all_exp_z; exp_z];
        all_teo_x = [all_teo_x; teo_x];
        all_teo_y = [all_teo_y; teo_y];
        all_teo_z = [all_teo_z; teo_z];
        
        % Create a new figure for normalized error
        figure;
        plot(err_norm, 'm', 'DisplayName', 'Normalized Error');
        title(['Experiment ' num2str(i) ': Normalized Error']);
        xlabel('Sample Index');
        ylabel('Error Norm');
        legend;
        grid on;
    end
    
    % Create a new figure for 3D plot of all data
    figure;
    plot3(all_exp_x, all_exp_y, all_exp_z, 'r', 'DisplayName', 'ROI');
    hold on;
    plot3(all_teo_x, all_teo_y, all_teo_z, 'b', 'DisplayName', 'Aruco');
    title('3D Plot of All ROI and Aruco Coordinates');
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    zlabel('Z Coordinate');
    legend;
    grid on;
    view(3);