%% Las Project - Alberto Dallolio

% Creates a driver object. It can drive the robot through randomly
% chosen waypoints.
% Whenever robot.calldriver function is used, the driver is connected to the
% robot. The driver's query method is invoked on every call from robot.onestep() method.

%% PROPERTIES

%  - goal            Current goal coordinate
%  - robot           The Robot object
%  - dim             Dimensions of the operational space (2x1) in meters
%  - speed           Speed in meters/seconds
%  - proximity       Proximity to goal
%  - goal_handle     Graphics handle for goal
%  - d_prev          Distance from previous point, to be set=Inf so that it will not be chosen anymore
%  - randstream      Random stream of points

%% METHODS

%  - Path       Object constructor
%  - init       Resets the random points generator
%  - query      Returns speed and steer angle to the next goal
%  - query_()   Returns speed and steer angle to the next goal in CHOSEN MODE
%  - visualize  Visualizes the driver
%  - setgoal()  Invoked from query() or query_() to compute a new goal

%% Implementation

classdef Path < handle
    
    properties
        goal         % current goal
        goal_handle  % graphics handle for goal
        robot        % the vehicle we are driving
        dim
        speed
        proximity    % proximity to goal before
        d_prev       % distance from previous point, to be set=Inf so that it will not be chosen anymore
        randstream   % random stream of points
    end
    
    methods
        
        function driver = Path(dim,varargin)
            % Creates a driver object
            
            % OPTIONS
            % 'speed',S      Speed along the path (default 1m/s, very slow!).
            % 'dist',D       Distance from goal at which next goal is chosen.
            
            driver.dim = dim;
            
            options.speed = 1;
            options.dist = 0.06 * dim;   % distance from goal at which next goal is chosen.
            options = tb_optparse(options, varargin);   %standard option parser, http://www.petercorke.com/RTB/r9/html/tb_optparse.html
            
            driver.speed = options.speed;
            driver.proximity = options.dist;
            driver.d_prev = Inf;
            driver.randstream = RandStream.create('mlfg6331_64'); %RandStream.create('mlfg6331_64'); %Multiplicative lagged Fibonacci generator, http://it.mathworks.com/help/matlab/ref/randstream.list.html
        end
        
        function init(driver)
            % Resets random number generator.
            % This enables the sequence of random waypoints to be repeated.
            
            driver.goal = [];
            driver.randstream.reset();
        end
        
        function visualize(driver)
            clf
            d = driver.dim;
            axis([-d d -d d]);
            hold on
            xlabel('x');
            ylabel('y');
        end
        
        function setgoal(driver)   % private method, invoked from query() to compute a new waypoint
            %r = driver.randstream.rand(2,1);
            a=[-1 1];
            b=a(1 + floor(rand * length(a)));
            driver.goal(1,1) = driver.randstream.rand(1,1)*driver.dim*b; 
            b=a(1 + floor(rand * length(a)));
            driver.goal(2,1) = driver.randstream.rand(1,1)*driver.dim*b;
            %driver.goal=driver.goal';
            fprintf('goal: (%.1f %.1f)\n', driver.goal);
            %plot(driver.goal(1), driver.goal(2), '*')
            %clear driver.h_goal;
           
            %% Questa parte sarebbe bello poterla decommentare, ma dopo una run mi da un errore che non riesco a risolvere
%             if isempty(driver.goal_handle)
%               driver.goal_handle = plot(driver.goal(1), driver.goal(2), '*');
%             else
%               set(driver.goal_handle, 'Xdata', driver.goal(1), 'Ydata', driver.goal(2))
%             end
        
        end
        
        function [speed, steer] = query(driver)
            % Computes the speed and angle to waypoint
            %
            % It returns the speed and steer angle to
            % drive the robot towards the next goal/waypoint. When the robot is
            % within driver.proximity a new point is selected.
            
            if isempty(driver.goal)
                driver.setgoal()
            end
            speed = driver.speed;
            goal_heading = atan2(driver.goal(2)-driver.robot.x(2), ...
                driver.goal(1)-driver.robot.x(1));
            d_heading = angdiff(goal_heading, driver.robot.x(3));  % the result of angdiff in the interval [-pi pi].
            steer = d_heading;
            
            % if the robot is close to a goal point, choose the next one
            d = sqrt(sum((driver.robot.x(1:2) - driver.goal).^2));
            if d < driver.proximity
                driver.setgoal();
            elseif d > driver.d_prev
                driver.setgoal();
            end
            driver.d_prev = d;
            %plot(driver.goal(1), driver.goal(2), '*')
        end
        
        function [speed, steer] = query_(driver,a)
            % Computes the speed and angle to waypoint
            % Works the same but in CHOSEN MODE.
            
            driver.goal=a;   % indeed a is the actual goal chosen manually by the user in robot.run_chosen().
            speed = driver.speed;
            goal_heading = atan2(driver.goal(2)-driver.robot.x(2), ...
                driver.goal(1)-driver.robot.x(1));
            d_heading = angdiff(goal_heading, driver.robot.x(3));  % the result of angdiff in the interval [-pi pi).
            steer = d_heading;
            driver.goal=driver.goal';
            %plot(driver.goal(1), driver.goal(2), '*');
            % if the robot is close to a goal point, choose the next one
%             d = colnorm(driver.robot.x(1:2) - driver.goal);
%             if d < driver.proximity
%                 driver.goal=a(c+1,:);
%             elseif d > driver.d_prev
%                 driver.goal=a(c+1,:);
%             end
%             driver.d_prev = d;
%             plot(driver.goal(1), driver.goal(2), '*')
        end
  
    end % methods
    
end % classdef
