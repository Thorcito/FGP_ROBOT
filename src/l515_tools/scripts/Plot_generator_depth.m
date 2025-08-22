%Funcion para calcualar y plotear el error en las medidas de profundidad
%capturadas por un sensor LiDar de la camara L515 y comparadonlas con
%medidas manuales realizadas por una cinta metrica. 

function Plot_generator_depth()
    data = readmatrix("Calibration.csv");
    disp(data);

    teo_depth = data(:,3);
    exp_depth = data(:,4);
    exp_std = data(:,6);
    error = teo_depth - exp_depth; % Calculate the error between theoretical and experimental depths
    % Plotting the theoretical and experimental depths with error bars
    figure;
    hold on; % Hold on to plot multiple datasets
    plot(exp_depth, teo_depth, 'o', 'Color', 'b', 'DisplayName', 'Theoretical Depth'); % Theoretical Depth
    errorbar(exp_depth, exp_depth, exp_std, 'horizontal', 's', 'Color', 'r', 'MarkerFaceColor', 'r', 'DisplayName', 'Experimental Depth'); % Experimental Depth with error
    xlabel('Experimental Depth');
    ylabel('Theoretical Depth');
    title('Depth Comparison with Error Bars');
    legend show; % Show legend to differentiate datasets
    grid on;

    % Adding error text boxes
    for i = 1:length(exp_depth)
        text(exp_depth(i), teo_depth(i)+0.002, sprintf('%.4f', error(i)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end
    
    hold off; % Release the hold




    
