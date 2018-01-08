%% Las Project - Alberto Dallolio

% Particle filter based on MonteCarlo localisation for estimating the robot pose on the basis of
% odometry and observations of (known) landmarks.


%% Particle Filtering - Main Concepts

% Objective: track a variable of interest as it evolves in time (typically with non-gaussian and multimodal pdf).
% How: construct a sample-based representation of the entire pdf.
% Multiple copies (particles) of the variable of interest are used, each one associated to a weight.
% An estimate of that variable is obtained by the weighted sum of all the particles.
% Recursive algorithm: prediction, update. After each action, every particle is modified
% according to the existing model (prediction), including a noise addition.
% Then each particle's wheight is re-evaluated on the lates sensory information available (update).
% The particles with very small weight are discarded (resampling phase).
% In our case the variable of interest is the robot pose [x y theta]. Indeed, each particle
% consists of a copy of the robot pose and a weight, that indicates how much it contributes
% to the overall estimation.


%% PROPERTIES

% ** Already built **
%  - robot                    robot object
%  - sensor                   sensor object

% ** New **
%  - particles_num            number of particles
%  - x                        particle states (particles_num x 3)
%  - weight                   particle weights (particles_num x 1)
%  - mean                     mean of the population
%  - std                      standard deviation of the population
%  - W                        model noise
%  - Q                        likelihood noise
%  - w0                       offset in likelihood model
%  - dim                      maximum xy dimension
%  - history                  vector of structs that hold the detailed information from each time step


%% METHODS

% ** Public Methods **
% - Filter      filter constructor
% - init        filter initialization
% - run         run the filter
% - step1       performs one step (iteration) of the filter operations
% - plot_pdf    display particle distribution




classdef Filter < handle        % so it is a reference object
    
    properties
        robot
        sensor
        particles_num
        x                       % particle states; nparticles x 3
        weight                  % particle weights; nparticles x 1
        mean                    % mean of the particle population
        deviation               % standard deviation of the particle population
        W                       % covariance of noise added to state at each step
        Q                       % covariance of likelihood model
        history
        keephistory
        dim                     % maximum xy dimension
        h                       % graphics handle for particles
        randstream
        w0
        x0                      % initial particle distribution
    end % properties
    
    methods
        
        function filter = Filter(robot, sensor, num, varargin)
            % Particle filter constructor
            % filter = Filter(robot, sensor, num, options) is a particle
            % filter that estimates the state of the robot with a sensor.
            % W is the noise added to the particles at each step.
            % Q is the covariance used in the
            % sensor likelihood model, and num is the number of particles.
            %
            % OPTIONS
            % 'reset'       Reset random number stream.
            
            
            filter.robot = robot;
            filter.sensor = sensor;
            filter.W = [0.1, 0.1, 1*pi/180].^2;
            filter.Q = [0.1 0.1];
            filter.particles_num = num;
            
            filter.dim = sensor.map.dim;
            filter.history = [];
            filter.x = [];
            filter.weight = [];
            filter.x0 = [];
            
            option.reset = false;
            option.history = true;
            option.x0 = [];
            
            option = tb_optparse(option, varargin);
            
            filter.keephistory = option.history;
            
            filter.randstream = RandStream.create('mlfg6331_64');
            
            % reset the random number stream if required
            if option.reset
                filter.randstream.reset();
            end
            
        end
        
        
        function init(filter)
            % Initializes the particle filter: initializes the distribution
            % and clears the history.
            % Particles are thrown in the map with a random distribution.
            % All the weights are set to unity.
            
            filter.robot.init();
            filter.history = [];
            
            % initial particle distribution: uniformly and randomly
            % distribution over the map area and heading angles
            
            % WAY 1
            %dist = makedist('Uniform');    %generation of the uniform distribution
            %filter.x = (2*random(dist,filter.particles_num,3)-1) * diag([filter.dim, filter.dim, pi]);
            
            % WAY 2
            filter.x = (2*filter.rand(filter.particles_num,3)-1) * diag([filter.dim, filter.dim, 180/pi]);
            
            filter.weight = ones(filter.particles_num, 1);   % all weights are initialized at 1
            filter.mean = [];
            filter.deviation = [];
            
        end
        
        function run(filter, tot, varargin)
            % Runs the particle filter for tot timesteps
            
            filter.init();
            filter.sensor.map.plot();
            a = axis;
            a(5:6) = [-pi pi];      % add dimension for the plot
            axis(a)
            zlabel('angle (rad)');  % the plot is in 3D: during the filter run the user can rotate the map.
            
            % display the initial particles
            filter.h = plot3(filter.x(:,1), filter.x(:,2), filter.x(:,3), 'b.');  % throw the particles!
            
            filter.robot.plot_robot();                                            % throw the robot!
            
            % iterate over time
            for i=1:tot
                filter.step1();
            end
            
            %display('Filter accuracy in percentage:');
            
            hold on
            filter.robot.plot_real('not')
            filter.plot_est
            grid
            
            x = 1:tot;
            y11 = filter.mean(:,1);
            y21 = filter.mean(:,2);
            y31 = filter.mean(:,3);
            
            y12 = filter.robot.x_past(:,1);
            y22 = filter.robot.x_past(:,2);
            y32 = filter.robot.x_past(:,3);
            
            figure 
            subplot(3,1,1);
            plot(x,y11,'r');
            hold on
            plot(x,y12,'b');
            grid
            title('X coordinate')
            legend('estimated value','real value')
            
            subplot(3,1,2);
            plot(x,y21,'r');
            hold on
            plot(x,y22,'b');
            grid
            title('Y coordinate')
            legend('estimated value','real value')
            
            subplot(3,1,3);
            plot(x,y31,'r');
            hold on
            plot(x,y32,'b');
            grid
            title('Heading angle (radians)')
            legend('estimated value','real value')
            
        end
        
        function step1(filter)
            
            odo = filter.robot.onestep();                       % move the robot one step ahead and return the odometry
            
            filter.predict(odo);                                % update the particles based on odometry
            [z,c] = filter.sensor.reading();                    % get a sensor reading
            %z=z*180/pi;
            
            if ~isnan(c)                                       % there's always a reading: this condition is useless!
                filter.update(z,c);                            % predict observation
                filter.resample();                             % pick particles
            end
            % the estimate is (simply) the mean of the particles
            m = mean(filter.x);
            filter.mean = [filter.mean; m];
            s = std(filter.x);
            
            % std is more complex for angles
            s(3) = sqrt(sum(angdiff(filter.x(:,3), m(3)).^2)) / (filter.particles_num-1);
            filter.deviation = [filter.deviation; s];
            
            % display the updated particles
            set(filter.h, 'Xdata', filter.x(:,1), 'Ydata', filter.x(:,2), 'Zdata', filter.x(:,3));
            
            filter.robot.plot_robot();
            drawnow
            
            
            
%             if filter.keephistory
%                 hist = [];
%                 hist.mean = filter.x;
%                 hist.w = filter.weight;
%                 filter.history = [filter.history hist];
%             end
        end
        
        function plot_pdf(filter)
            % Plots particles as a PDF
            clf
            %hold on
            for p = 1:filter.particles_num
                a = filter.x(p,:);
                plot3([a(1) a(1)], [a(2) a(2)], [0 filter.weight(p)]);
                hold on
                % series of vertical line segments of height equal to particle weight
            end
            grid
        end
  
        function plot_est(filter, varargin)
            % Plots estimated vehicle position
            plot(filter.mean(:,1), filter.mean(:,2), varargin{:});
            grid
        end
        
        
        
    end % methods
    
    methods%(Access=protected)                   % private methods: access from methods in class or subclasses
        %% Prediction
        function predict(filter, odo)
            % Updates the particle state based on odometry and a random perturbation
            
            for i=1:filter.particles_num
                x = filter.robot.pippo(filter.x(i,:), odo)'+ sqrt(filter.W)*filter.randn(3,1); 
                % The filter needs a noise model to "drift" the particles at each step, that
                % is the hypotheses are randomly moved to model the effect of uncertainty in the
                % robot's pose. Values are consistent with robot model
                % noise.
                % Pippo computes the next state given the current state and the odometry

                % add the perturbation to the next state
                x(3) = angdiff(x(3));       % re-computes the theta in the proper range.
                %x(3,:)=x(3,:)*180/pi;
                filter.x(i,:) = x;
            end
            
        end
        
        %% Update
        function update(filter, z, a)
            % Predicts the observation and scores the particles
            % Computes the expectation for each particle.
            % Computes how much it differs from what it actually is and finally assign that particle a weight.
            % The particle filter requires a likelihood function that maps
            % the error between expected and actual sensor observation to a weight
            
            for j=1:filter.particles_num
                
                z_predicted = filter.sensor.compute_bearing(filter.x(j,:),a);        % what I expect the observation for this particle is!
                %z_predicted=z_predicted*180/pi;
                
                innovation = angdiff(z, z_predicted);                                % how different it actually is! (innovation)
                %innovation = z-z_predicted;
                
                filter.weight(j) = exp(-0.5*innovation'*inv(filter.Q(1))*innovation) + 0.05;        %wgn(1,1,0)  % the weight is never zero!
                
                % Get likelihood (new importance). Assume Gaussian pdf.
                % If the predicted observation is very different from
                % actual observation this value will be low: this particle is not very good at predicting the observation.
                % A lower score means it is less likely to be selected for
                % the next generation!
                
            end
            %filter.weight = filter.weight/norm(filter.weight,2);
        end
        
        %% Resampling
        function resample(filter)
            % Selects particles based on their weights
            
            % Particles with large weights will occupy a greater portion of
            % the y axis in a cumulative plot (in the vertical axis)
            cdf = cumsum(filter.weight)/sum(filter.weight);   % cumulative sum/sum
            % A particle set is a discrete distribution: there is no density that could be associated with a set of particles.
            % The convergence is over the cumulative distribution function.
            
            % query points for the interpolation are
            % chosen randomly!!
            select  = filter.rand(filter.particles_num,1);
            
            % find the particle that corresponds to each y value
            NewGeneration = interp1(cdf, 1:filter.particles_num, select, 'nearest', 'extrap'); 
            % NewGeneration is a vector containing elements corresponding to the elements of
            % select and determined by interpolation within vectors cdf and 1:filter.particles_num.
            % cdf specifies the points at which the particles are given.
            % nearest neighbor interpolation method: the interpolated
            % value at a query point is the one at the nearest
            % sample grid point.
            % NewGeneration contains the elements corresponding to select
            % obtained by interpolating cdf and 1:particles_num. cdf
            % specifies the point at which the particle num is given.
            
            filter.x = filter.x(NewGeneration,:);   % copy selected particles for next generation
        end
        
        function r = rand(filter, varargin)
            r = filter.randstream.rand(varargin{:});
        end
        
        function r = randn(filter, varargin)
            r = filter.randstream.randn(varargin{:});
        end
    end % private methods
end




