function [fig_vel, fig_eul, ...
          fig_rates, fig_rpm] = plot_multirotor_states(logsout, options)
    % Generate and save plots of the multirotor states (body frame
    % velocity, Euler angles, angular rates) and controls (commanded and
    % actual motor speeds).
    %
    % Arguments
    % ---------
    % logsout : (:, 1) Simulink.SimulationData.Dataset
    %   Data output from BAM simulation(s), for example
    %       logsout = sim('BAM.slx').logsout;
    %
    % Keyword Arguments
    % -----------------
    % fontsize : int, default=16
    %   Font size to use in plots.
    % reference_color : (1, 3) double, default = [0, 0, 0]
    %   Color for line plots of reference inputs.
    % multirotor_color : (:, 3) double, default = colororder()
    %   Colors for line plots of multirotor trajectories.
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
    % fig_vel : Figure
    %   Plots of multirotor body velocity components, u, v, w.
    % fig_eul : Figure
    %   Plots of multirotor attitude in Euler angles, phi, theta, psi.
    % fig_rates : Figure
    %   Plots of multirotor angular rates, p, q, r.
    % fig_rpm : Figure
    %   Plots of rotor commands and speeds.
    %
    % Author
    % ------
    % Tenavi Nakamura-Zimmerer
    % NASA Langley Research Center (LaRC),
    % Flight Dynamics Branch (D-317)
    arguments
        logsout (:, 1)
        options.fontsize (1, 1) double = 16
        options.reference_color (1, 3) double = [0, 0, 0]
        options.multirotor_color (:, 3) double = colororder()
        options.save_dir (1, :) char = ''
        options.save_filetype (1, :) char = 'pdf'
        options.save_args cell = {'ContentType', 'Vector'}
    end
    
    label_fmt = {'Interpreter','latex', 'fontsize', options.fontsize};
    
    n_traj = length(logsout);

    time = cell(1, n_traj);
    
    Vel_bIi = cell(1, n_traj);
    eta = cell(1, n_traj);
    Omeg_BIb = cell(1, n_traj);
    eng_cmd = cell(1, n_traj);
    eng_speed = cell(1, n_traj);

    for i = 1 : n_traj
        sim_data = logsout{i}.Values;
        
        time{i} = sim_data.RefInputs.pos_i.Time;
    
        % Body velocity
        Vel_bIi{i} = sim_data.EOM.InertialData.Vel_bIi.Data;
    
        % Euler angles
        eta{i} = rad2deg( ...
            [sim_data.EOM.WorldRelativeData.Euler.phi.Data, ...
             sim_data.EOM.WorldRelativeData.Euler.theta.Data, ...
             sim_data.EOM.WorldRelativeData.Euler.psi.Data]);

        % Angular rates
        Omeg_BIb{i} = rad2deg(sim_data.EOM.InertialData.Omeg_BIb.Data);

        % Motor commands and speeds
        eng_cmd{i} = sim_data.Misc.EngCmd.EngCmd.Data;
        eng_speed{i} = sim_data.Misc.EngCmd.EngSpeed.EngSpeed.Data;
    end

    % Velocity plots
    
    fig_vel = figure; hold on;

    labels = {'u', 'v', 'w'};
    names = {'Forward', 'Lateral', 'Down'};
    
    for i = 1 : 3
        subplot(3, 1, i)
        hold on; box on; grid on; zoom on;

        for j = 1 : n_traj
            plot(time{j}, Vel_bIi{j}(:, i), 'color', ...
                 options.multirotor_color(1, :), 'linewidth', 2)
        end
    
        xlabel('time (s)', label_fmt{:})
        ylabel(sprintf('$%s$ (m/s)', labels{i}), label_fmt{:}) 
        title(sprintf('{\\bf Body %s Velocity}', names{i}), label_fmt{:}) 
    end

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['Velocity.', options.save_filetype]);
        exportgraphics(fig_vel, filepath, options.save_args{:})
    end

    % Attitude plots

    fig_eul = figure; hold on;

    labels = {'phi', 'theta', 'psi'};
    names = {'Roll', 'Pitch', 'Yaw'};
    
    for i = 1 : 3
        subplot(3, 1, i)
        hold on; box on; grid on; zoom on;

        for j = 1 : n_traj
            plot(time{j}, eta{j}(:, i), 'color', ...
                 options.multirotor_color(1, :), 'linewidth', 2)
        end
    
        xlabel('time (s)', label_fmt{:})
        ylabel(sprintf('$\\%s$ (deg)', labels{i}), label_fmt{:}) 
        title(sprintf('{\\bf %s}', names{i}), label_fmt{:}) 
    end

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['EulerAngles.', options.save_filetype]);
        exportgraphics(fig_eul, filepath, options.save_args{:})
    end

    % Angular rate plots

    fig_rates = figure; hold on;

    labels = {'p', 'q', 'r'};
    names = {'Roll', 'Pitch', 'Yaw'};
    
    for i = 1 : 3
        subplot(3, 1, i)
        hold on; box on; grid on; zoom on;

        for j = 1 : n_traj
            plot(time{j}, Omeg_BIb{j}(:, i), 'color', ...
                 options.multirotor_color(1, :), 'linewidth', 2)
        end
    
        xlabel('time (s)', label_fmt{:})
        ylabel(sprintf('$%s$ (deg/s)', labels{i}), label_fmt{:}) 
        title(sprintf('{\\bf %s Rate}', names{i}), label_fmt{:}) 
    end

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['AngularRates.', options.save_filetype]);
        exportgraphics(fig_rates, filepath, options.save_args{:})
    end

    % Control input plots

    fig_rpm = figure; hold on;
    resizeFig(fig_rpm, [1, 1.4] .* fig_rpm.Position(3:4))

    for i = 1 : 4
        subplot(4, 1, i)
        hold on; box on; grid on; zoom on;

        for j = 1 : n_traj
            plots = [plot(time{j}, eng_cmd{j}(:, i), 'color', ...
                          options.reference_color, 'linewidth', 2, ...
                          'DisplayName', 'command'), ...
                     plot(time{j}, eng_speed{j}(:, i), '--', 'color', ...
                          options.multirotor_color(1, :), 'linewidth', ...
                          2, 'DisplayName', 'actual')];
        end
    
        xlabel('time (s)', label_fmt{:})
        ylabel(sprintf('$\\omega_%d$ (rpm)', i), label_fmt{:}) 
        title(sprintf('{\\bf Motor %d}', i), label_fmt{:})
        legend(plots, 'Location', 'best', label_fmt{:})
    end

    if ~isempty(options.save_dir)
        filepath = fullfile(options.save_dir, ...
                            ['MotorSpeeds.', options.save_filetype]);
        exportgraphics(fig_rpm, filepath, options.save_args{:})
    end
end
