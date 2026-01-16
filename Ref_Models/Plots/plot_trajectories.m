function [fig_traj, fig_grnd, fig_alt] = plot_trajectories( ...
        logsout, bball_pwcurve, options)
    % Generate and save plots of the multirotor and baseball trajectories
    % in 3d, ground track, and altitude.
    %
    % Arguments
    % ---------
    % logsout : (:, 1) Simulink.SimulationData.Dataset
    %   Data output from BAM simulation(s), for example
    %       logsout = sim('BAM.slx').logsout;
    % bball_pwcurve : (:, 3) cell, optional
    %   If desired, description of baseball trajectories compatible for
    %   evaluation as a piecewise Bezier polynomial with evalPWCurve.
    %
    % Keyword Arguments
    % -----------------
    % fontsize : int, default=16
    %   Font size to use in plots.
    % reference_color : (1, 3) double, default = [0, 0, 0]
    %   Color for line plots of reference inputs.
    % multirotor_color : (1, 3) double, default = [0, 0.4470, 0.7410]
    %   Colors for line plots of multirotor trajectories.
    % baseball_color : (1, 3) double, default = [0.8500, 0.3250, 0.0980]
    %   Colors for line plots of baseball trajectories.
    % baseball_plot_dt : (1, 1) double, default=0.01
    %   Desired time discretization of baseball trajectory.
    % save_dir : char, optional
    %   Directory where plots should be saved. Plots are not saved if
    %   save_dir is empty.
    % save_filetype : char, default='pdf'
    %   Format in which plots should be saved.
    % save_args : cell, default={'ContentType', 'Vector'}
    %   Arguments to pass to exportgraphics when saving figures.
    %
    % Returns
    % -------
    % fig_traj : Figure
    %   3d plot of the multirotor trajectory and reference trajectory.
    % fig_grnd : Figure
    %   2d ground track of the multirotor and reference ground track.
    % fig_alt : Figure
    %   Flown and reference altitude profiles vs. time.
    %
    % Author
    % ------
    % Tenavi Nakamura-Zimmerer
    % NASA Langley Research Center (LaRC), 
    % Flight Dynamics Branch (D-317)
    arguments
        logsout (:, 1)
        bball_pwcurve (:, 3) cell = {}
        options.plot_reference (1, 1) logical = true
        options.fontsize (1, 1) double = 16
        options.reference_color (1, 3) double = [0, 0, 0]
        options.multirotor_color (1, 3) double = [0, 0.4470, 0.7410]
        options.baseball_color (1, 3) double = [0.8500, 0.3250, 0.0980]
        options.baseball_plot_dt (1, 1) double {mustBePositive} = 0.01
        options.save_dir (1, :) char = ''
        options.save_filetype (1, :) char = 'pdf'
        options.save_args cell = {'ContentType', 'Vector'}
    end
    
    label_fmt = {'Interpreter','latex', 'fontsize', options.fontsize};

    plot_bball = ~isempty(bball_pwcurve);

    n_traj = length(logsout);
    n_bballs = size(bball_pwcurve, 1);

    time = cell(1, n_traj);
    
    if options.plot_reference
        traj_linestyle = '--';
        traj_plot_idx = 2;
        bball_plot_idx = 3;

        x_des = cell(1, n_traj);
        y_des = cell(1, n_traj);
        h_des = cell(1, n_traj);
    else
        traj_linestyle = '-';
        traj_plot_idx = 1;
        bball_plot_idx = 2;
    end
    
    x = cell(1, n_traj);
    y = cell(1, n_traj);
    h = cell(1, n_traj);

    t_bball = cell(1, n_bballs);
    bball_pos = cell(1, n_bballs);

    for i = 1 : n_traj
        sim_data = logsout{i}.Values;
        
        time{i} = sim_data.RefInputs.pos_i.Time;
    
        if options.plot_reference
            x_des{i} = sim_data.RefInputs.pos_i.Data(:, 1);
            y_des{i} = sim_data.RefInputs.pos_i.Data(:, 2);
            h_des{i} = -sim_data.RefInputs.pos_i.Data(:, 3);
        end
    
        x{i} = sim_data.EOM.InertialData.Pos_bii.Data(:, 1);
        y{i} = sim_data.EOM.InertialData.Pos_bii.Data(:, 2);
        h{i} = -sim_data.EOM.InertialData.Pos_bii.Data(:, 3);
    end

    if plot_bball
        for i = 1 : n_bballs
            % Extract time and position from Bernstein polynomial data
            t_bball_i = [bball_pwcurve{i}{1}.tint(1), ...
                         bball_pwcurve{i}{end}.tint(end)];
            t_bball{i} = linspace( ...
                t_bball_i(1), t_bball_i(2), ...
                (t_bball_i(2) - t_bball_i(1)) / options.baseball_plot_dt);
            bball_pos{i} = evalPWCurve(bball_pwcurve(i, :), t_bball{i}, 0);
        end
    end

    % 3D plot
    
    fig_traj = figure; hold on; box on; grid on; zoom on;
    
    for i = 1 : n_traj
        if options.plot_reference
            plots(1) = plot3(x_des{i}, y_des{i}, h_des{i}, 'color', ...
                             options.reference_color, 'linewidth', 2, ...
                             'DisplayName', 'reference');
        end

        plots(traj_plot_idx) = plot3(x{i}, y{i}, h{i}, traj_linestyle, ...
                                     'color', options.multirotor_color, ...
                                     'linewidth', 2, ...
                                     'DisplayName', 'flown');
    end

    if plot_bball
        for i = 1 : n_bballs
            plots(bball_plot_idx) = plot3( ...
                bball_pos{i}(:, 1), bball_pos{i}(:, 2), ...
                -bball_pos{i}(:, 3), ':', 'color', ...
                options.baseball_color, 'linewidth', 2, 'DisplayName', ...
                'baseball');
        end
    end
    
    ax = gca();
    ax.YDir = 'reverse';
    
    xlabel('North (m)', label_fmt{:})
    ylabel('East (m)', label_fmt{:})
    zlabel('Altitude (m)', label_fmt{:})

    if options.plot_reference || plot_bball
        legend(plots, 'Location', 'best', label_fmt{:})
    end

    if plot_bball
        title('{\bf Multirotor and baseball trajectories}', label_fmt{:})
    elseif n_traj > 1
        title('{\bf Multirotor trajectories}', label_fmt{:})
    else
        title('{\bf Multirotor trajectory}', label_fmt{:})
    end

    view(-15,30)
    axis equal

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['Trajectory.', options.save_filetype]);
        exportgraphics(fig_traj, filepath, options.save_args{:})
    end

    % Ground track plot
    
    fig_grnd = figure; hold on; box on; grid on; zoom on;
    
    for i = 1 : n_traj
        if options.plot_reference
            plots(1) = plot(y_des{i}, x_des{i}, 'color', ...
                            options.reference_color, 'linewidth', 2, ...
                            'DisplayName', 'reference');
        end

        plots(traj_plot_idx) = plot(y{i}, x{i}, traj_linestyle, ...
                                    'color', options.multirotor_color, ...
                                    'linewidth', 2, ...
                                    'DisplayName', 'flown');
    end

    if plot_bball
        for i = 1 : n_bballs
            plots(bball_plot_idx) = plot( ...
                bball_pos{i}(:, 2), bball_pos{i}(:, 1), ':', 'color', ...
                options.baseball_color, 'linewidth', 2, 'DisplayName', ...
                'baseball');
        end
    end
    
    xlabel('East (m)', label_fmt{:})
    ylabel('North (m)', label_fmt{:})

    if options.plot_reference || plot_bball
        legend(plots, 'Location', 'best', label_fmt{:})
    end

    if plot_bball || n_traj > 1
        title('{\bf Ground tracks}', label_fmt{:})
    else
        title('{\bf Ground track}', label_fmt{:})
    end

    axis equal

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['GroundTrack.', options.save_filetype]);
        exportgraphics(fig_grnd, filepath, options.save_args{:})
    end

    % Altitude plot

    fig_alt = figure; hold on; box on; grid on; zoom on;
    
    resizeFig(fig_alt, [1, 0.6] .* fig_alt.Position(3:4))
    
    for i = 1 : n_traj
        if options.plot_reference
            plots(1) = plot(time{i}, h_des{i}, 'color', ...
                      options.reference_color, 'linewidth', 2, ...
                      'DisplayName', 'reference');
        end

        plots(traj_plot_idx) = plot(time{i}, h{i}, traj_linestyle, ...
                                    'color', options.multirotor_color, ...
                                    'linewidth', 2, 'DisplayName', ...
                                    'flown');
    end

    if plot_bball
        for i = 1 : n_bballs
            plots(bball_plot_idx) = plot( ...
                t_bball{i}, -bball_pos{i}(:, 3), ':', 'color', ...
                options.baseball_color, 'linewidth', 2, 'DisplayName', ...
                'baseball');
        end
    end
    
    xlabel('time (s)', label_fmt{:})
    ylabel('$h$ (m)', label_fmt{:})
    title('{\bf Altitude}', label_fmt{:})

    if options.plot_reference || plot_bball
        legend(plots, 'Location', 'best', label_fmt{:})
    end

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['Altitude.', options.save_filetype]);
        exportgraphics(fig_alt, filepath, options.save_args{:})
    end
end
