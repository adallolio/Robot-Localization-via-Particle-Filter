%% Las Project - Alberto Dallolio

% Robot class contains properties and functions related to the Differential Drive Robot.
% The DDR is a triangle whose features can be set (speed, steering angle, dimensions, etc).
% Properties and methods are listed below.

% CHOSEN MODE: the chosen mode is a simulation modality thanks to which the
% robot's path can be chosen by the user. Once the simulation is started
% the user is asked to specif the points on the map the robot has to go through. 
% Then, the robot performs the motion. The duration of the simulation strictly depends on
% the number of points provided as inputs. Indeed, a proper function is
% responsible for the computation of the number of timesteps required for
% the robot to reach one point from the previous, depending on its speed
% and on the distances between points.

%% PROPERTIES

%   - x                         real robot state (x,y,a)
%   - x0                        initial state (0,0,0)
%   - x_past                    history of the robot state
%   - dim                       dimension of the robot
%   - l                         lenght of the robot
%   - steerlim                  maximum steering angle
%   - maxspeed                  maximum speed (m/s)
%   - odometry                  distance moved in the last interval (2x1)
%   - V                         covariance of the odometry (squared 2x3)
%   - dt                        sampling interval
%   - driver                    describes the controls moving the robot

%% METHODS

%   - Robot(varargin)                                       robot object contructor (options can be specified)
%   - calldriver(robot, driver)                             links the driver to the robot object, input
%                                                           driver comes from the Path generation

%   - update(robot, u)                                      updates the robot state on the base of
%                                                           controls and iutputs the odometry

%   - onestep()                                             updates the robot state for one timestep (returns the odometry, noisy if V is present)
%   - onestep_()                                            updates the robot state for one timestep (CHOSEN MODE)
%                                                           (returns the odometry, noisy if V is present)

%   - pippo()                                               predicts next state thanks to the odometry
%   - control()                                             computes the robot control inputs invoking the driver object
%   - control_()                                            computes the robot control inputs invoking
%                                                           the driver object (CHOSEN MODE)

%   - run_rand(robot,numsteps)                              runs a random simulation for NUM steps.
%                                                           Points are chosen randomly in Path.

%   - run_chosen(robot,points)                              runs a simulation in which the user can
%                                                           choose manually the points on the map the robot has to go through.



%% Implementation

classdef Robot < handle
    
    properties %(Access = public)
        
        % Robot State
        x                % real state (x,y,c)
        x_n              % noisy state (x,y,c)
        x0               % initial state
        x_past           % state history
        x_past_n         % noisy state history
        
        
        % Robot properties
        %base           % Wheels distance, not used
        dim             % Dimension of the robot
        l               % Length of robot
        steerlim        % Steering wheel limit
        %R              % True pose [x y c], not used
        %r              % Estimated pose [x y c]
        %q              % True system noise, not used
        maxspeed        % Maximum speed (m/s)
        odometry        % Distance moved in the last interval (2 x 1)
        odometry_n      % Noisy odometry
        model_noise     % Noise of the model
        sensor_noise    % Odometry covariance (2 x 2)
        dt              % Sample interval
        driver
    end
    
    methods
        
        function robot = Robot(varargin)
            % Object constructor
            %
            % V is a (2 x 2) matrix corresponding to the odometry vector [dx dtheta].
            % Varargin can be chosen among some options:
            %
            % OPTIONS
            % 'maxsteer',A    Steering angle limited to -A to +A (default 0.6 rad)
            % 'maxv',B        Maximum speed (default 4m/s)
            % 'L',L           Wheel base (default 1m)
            % 'x0',x0         Initial state (default (0,0,0))
            % 'dt',T          Time interval
            % 'dim',C         Robot size as fraction of plot window (default 0.2)
            
            % basic options: can be changed defining directly the robot
            % object parameters: robot = Robot('maxsteer',A,'maxv',B,'L',l,.....)
            
            options.maxsteer = 0.6;
            options.maxv = 4;
            options.L = 1;
            options.dim = 0.2;
            options.dt = 0.1;
            options.model_noise = wgn(1,3,1)*rand(1,1);   % model noise     
            options.x0 = zeros(3,1);
            options = tb_optparse(options, varargin);   % standard option parser, http://www.petercorke.com/RTB/r9/html/tb_optparse.html
            
            
            robot.model_noise = options.model_noise;
            robot.dt = options.dt;
            robot.steerlim = options.maxsteer;
            robot.maxspeed = options.maxv;
            robot.l = options.L;
            robot.x0 = options.x0(:);
            robot.dim = options.dim;
            robot.x_past = [];
            robot.x_past_n = [];
        end
        
        function NOTUSED(not)
            %         function robot = Robot()                         % Constructor method
            %             % Default values
            %             robot.base         =   0.75;
            %             robot.l            =   1;
            %             robot.R            =   [0, 0, 0]';
            %             robot.r            =   robot.R;
            %             robot.q            =   [0.02;pi/180];       % 2cm, 1 deg
            %             robot.u            =   [0.1; 0.02];
            %             robot.maxspeed     =   1.5;                 % 1.5m/s ~ 5km/h
            %             robot.min_turn     =   0.6;
            %         end
            %
            %
            %         function [handle]=draw(robot,pose)
            %             center=[pose(1);pose(2)];
            %             hold on;
            %
            %             width=robot.base;
            %             height=robot.length;
            %             theta = pose(3)*(pi/180);
            %             coords = [center(1)-(width/2) center(1)-(width/2)  center(1)+(width/2);...
            %                 center(2)-(height/2) center(2)+(height/2)  center(2)+(height/2)];
            %             Rot = [cos(theta) sin(theta);...
            %                 -sin(theta) cos(theta)];
            %             rot_coords = Rot*(coords-repmat(center,[1 3]))+repmat(center,[1 3]);
            %             axis([0,1,0,1]);
            %             handle=patch([rot_coords(1,1),rot_coords(1,2),rot_coords(1,3)], [rot_coords(2,1),rot_coords(2,2),rot_coords(2,3)],'black');
            %             plot(pose(1),pose(2),'.r');
            %             grid
            %
            %         end
            
            %         %% Return the vertices of a triangle of dimensions base times length.
            %         % the robot is centred at (x,y) and with orientation c.
            %         %
            %         % Inputs:
            %         %   r = [x y c]'        :   local coordinate frame of robot
            %         %   base                :   size of triangle base
            %         %   length              :   length of triangle
            %         %
            %         % Outputs:
            %         %   p = [x1 x2 x3 x4;
            %         %        y1 y2 y3 y4]	:   vertices of resulting triangle
            %         function p = computeTriangle(robot, strRobType, r_pos)
            %
            %             if nargin < 2
            %                 pose = robot.r;
            %             elseif nargin < 3 && strcmp(strRobType, 'true')
            %                 pose = robot.R;
            %             elseif nargin < 4
            %                 pose = r_pos;
            %             else
            %                 error('strRobType can only take the value "True"');
            %             end
            %
            %             p = zeros(2, 4);
            %
            %             p(:, 1) = [-robot.length/3; -robot.base/2];
            %             p(:, 2) = [2*robot.length/3; 0];
            %             p(:, 3) = [-robot.length/3; robot.base/2;];
            %
            %             for i = 1:3
            %                 p(:, i) = transToGlobal(pose, p(:, i));
            %             end
            %
            %            % The fourth point is equal to the first, so to close
            %            % the triangle when the robot is plotted!
            %             p(:, 4) = p(:, 1);
            %
            %         end
            %
            %         function steer(robot, Wpts)
            %             % Determine if current waypoint reached
            %             wpt = Wpts(:, robot.curr_wpt);
            %             % Distance from current waypoint
            %             dist = sqrt((wpt(1)-robot.R(1))^2 + (wpt(2)-robot.R(2))^2);
            %             if dist < robot.min_turn
            %                 if robot.curr_wpt < size(Wpts, 2)
            %                     robot.curr_wpt = robot.curr_wpt + 1;
            %                 else
            %                     robot.curr_wpt = 1; % Go back to the start
            %                 end
            %                 wpt = Wpts(:, robot.curr_wpt);
            %             end
            %             % Change in robot orientation to face current waypoint
            %             dc = getPiAngle(atan2(wpt(2)-rob.R(2), wpt(1)-rob.R(1))-rob.R(3));
            %             % Amount by which to perturb current robot control:
            %             ddc = dc - robot.u(2);
            %             du = [0; ddc];
            %             robot.setControl(du);
            %         end
            %
            %         % Perturb control vector by an amount du = [accel_x; accel_c].
            %         function u = setControl(robot, du)
            %             u = robot.u + du;
            %             % Make sure that angular control is between -pi and pi
            %             u(2) = getPiAngle(u(2));
            %             if abs(u(1)) > rob.maxspeed
            %                 % Adjust speed so that max speed isn't exceeded
            %                 u(1) = sign(u(1)) * robot.maxspeed;
            %             end
            %             turning_radius = u(1)/u(2);     % Using formula r = v/omega
            %             if abs(turning_radius) < robot.min_turn_rad
            %                 % Adjust angular velocity so that min turning radius isn't
            %                 % exceeded
            %             	u(2) = sign(u(2)) * abs(u(1)) / robot.min_turn_rad;
            %             end
            %
            %             robot.u = u;
            %         end
            %     end
        end
        
        function init(robot, x0)
            % Resets the state of robot object
            % robot.init() sets the state robot.x=robot.x0, initializes the driver
            % object (if attached) and clears the history.
            
            if nargin > 1
                robot.x = x0(:);
                robot.x_n = x0(:);
            else
                robot.x = robot.x0;
                robot.x_n = robot.x0;
            end
            
            % Possible model noise
%             a = -1;
%             b = 1;
%             robot.model_noise = (b-a).*rand(1,3) + a;
            
            robot.x_past = [];
            robot.x_past_n = [];
%                 if ~isempty(robot.driver)
%                     robot.driver.init;
%                 end
        end
        
        function calldriver(robot, driver)
            % Adds a driver for the Robot
            % robot.adddriver(D) connects a driver object D to the robot. The driver
            % object has one public method:
            %        [speed, steer] = D.query();
            % that returns a speed and steer angle.
            
            robot.driver = driver;
            driver.robot = robot;
        end
        
        function [odometry] = update(robot, u)
            % Updates the robot state
            % odometry = robot.update(U) is the TRUE odometry value for
            % motion with U=[speed,steer]
            
            xp = robot.x; % previous state
            
            robot.x(1) = robot.x(1) + u(1)*robot.dt*cos(robot.x(3));
            robot.x(2) = robot.x(2) + u(1)*robot.dt*sin(robot.x(3));
            robot.x(3) = robot.x(3) + u(2)*robot.dt;
            odometry = [sqrt(sum((robot.x(1:2)-xp(1:2)).^2)) robot.x(3)-xp(3)];
            robot.odometry = odometry;  % odometry is saved as property odometry.
            
            
            robot.x_n(1) = robot.x(1) + robot.model_noise(:,1);
            robot.x_n(2) = robot.x(2) + robot.model_noise(:,2);
            robot.x_n(3) = robot.x(3) + robot.model_noise(:,3);
            robot.odometry_n = [sqrt(sum((robot.x_n(1:2)-xp(1:2)).^2)) robot.x_n(3)-xp(3)];
            robot.odometry_n = robot.odometry_n;
            
            robot.x_past = [robot.x_past; robot.x'];   % maintain history, appends new state to state history property x_past.
            robot.x_past_n = [robot.x_past_n; robot.x_n'];
            
        end
        
        function [odom] = onestep(robot, speed, steer, varargin)
            % Computes one timestep
            % odom = robot.onestep(SPEED, STEER) updates the robot state for one timestep
            % of motion at specified SPEED and STEER angle, and returns noisy odometry.
            % odom = robot.onestep() updates the robot state for one timestep of motion and
            % returns noisy odometry.  If a driver is linked then its QUERY() method
            % is invoked to compute speed and steer angle.  If no driver is attached
            % then speed and steer angle are assumed to be zero.
            
            if nargin < 2
                % get the control input to the vehicle from either passed demand or driver
                u = robot.control(varargin{:});
                % compute the true odometry and update the state
                odom = robot.update(u);
            else
                u = [speed,steer];
                odom = robot.update(u);
            end

        end
        
        function [odom] = onestep_(robot, a, speed, steer, varargin)
            % Computes one timestep
            % The same as robot.onestep() but works for CHOSEN MODE.
            
            if nargin < 3
                % get the control input to the vehicle from either passed demand or driver
                u = robot.control_(a,varargin{:});
                % compute the true odometry and update the state
                odom = robot.update(u);
            else
                u = [speed,steer];
                odom = robot.update(u);
            end

        end
        
        function nextx = pippo(~, x, odometry, w)
            % Predicts next state thanks to the odometry computed in
            % robot.update() function.
            % nextx = robot.pippo(x, odometry) is the predicted next state nextx (1x3) based on current
            % state x (1x3) and odometry odometry (1x2) = [distance, pointing_change].
            % nextx = robot.pippo(x, odometry, w) as above but with odometry noise W.
            
            if nargin < 4
                w = [0 0];
            end
            
            dd = odometry(1) + w(1); dc = odometry(2);
            
            % Next state based on odometry
            % thp = x(3) + dth;
            % nextx = zeros(1,3);
            % nextx(1) = x(1) + (dd + w(1))*cos(thp);
            % nextx(2) = x(2) + (dd + w(1))*sin(thp);
            % nextx(3) = x(3) + dth + w(2);
            %x=x';
            cp = x(:,3) + dc;
            nextx = x + [(dd+w(1))*cos(cp)  (dd+w(1))*sin(cp) ones(size(x,1),1)*dc+w(2)];
        end
        
        function u = control(robot, speed, steer)
            % Computes the control inputs to the robot
            % Generally U is a control input (1x2) = [speed,steer]
            % based on provided controls SPEED,STEER.
            % U = robot.control() as above but the query originates with a driver object if
            % one is attached. The query() method of driver class is invoked. If no driver is
            % attached then speed and steer angle are assumed to be zero.
            
            if nargin < 2
                % if no explicit query, and a driver is attached, use
                % it to provide query
                if ~isempty(robot.driver) %&& robot.driver.goal(1)==robot.x(1) && robot.driver.goal(2)==robot.x(2)
                    [speed, steer] = robot.driver.query();
                else
                    % no demand, play safe
                    speed = 0;
                    steer = 0;
                end
            end
            
            % ajdust the speed
            u(1) = min(robot.maxspeed, max(-robot.maxspeed, speed));
            
            % ajdust the steering angle
            u(2) = max(-robot.steerlim, min(robot.steerlim, steer));
        end
        
        function u = control_(robot, a, speed, steer)
            % Robot.control computes the control inputs to the robot
            % The same as robot.control() but works for CHOSEN MODE.
            
            if nargin < 3
                % if no explicit demand, and a driver is attached, use
                % it to provide demand
                if ~isempty(robot.driver) %&& robot.driver.goal(1)==robot.x(1) && robot.driver.goal(2)==robot.x(2)
                    [speed, steer] = robot.driver.query_(a);
                else
                    % no demand, play safe
                    speed = 0;
                    steer = 0;
                end
            end
            
            % cut the speed
            u(1) = min(robot.maxspeed, max(-robot.maxspeed, speed));
            
            % cut the steering angle
            u(2) = max(-robot.steerlim, min(robot.steerlim, steer));
        end
        
        function [p,z,all_classes] = run_rand(robot,numsteps,map,sensor)
            % Robot.run_rand runs the random robot simultation.
            % Random because the robot's path is computed inkvoking the
            % driver with query(), that choose the goal from a stream of
            % points according to the chosen mode of the RandStream list.
            % Whenever a goal is set the controls to actuate the robot
            % towards that are computed and the simulation keeps running.
            % Every time the robot is approaching a goal the next is chosen
            % by the driver, and once the former is reached the trajectory
            % changes towards the latter. The simulation ends once the
            % number of steps is over (numsteps). The outputs are the robot
            % odometry and the outputs of the bearing sensor: the bearings
            % to the landmarks and the color of the landmarks.
            
            if nargin < 3
                numsteps = 1000;
            end
            
            %robot.clear();
            if ~isempty(robot.driver)
                robot.driver.visualize();
            end
            
            all_classes=[];
            new=zeros(map.num,3);
            alfa=numsteps/4;
            z=zeros(numsteps,map.num);
            
            robot.init;
            hold on
            
            robot.visualize();
            map.plot;
            
            for i=1:numsteps
                robot.onestep();
                %sensor.reading();
                if i==floor(alfa) || i==floor(2*alfa) || i==floor(3*alfa) || i==floor(4*alfa)             %compute the CLASSES every numsteps/4 step
                    [z(i,:),new(:,1:3)] = sensor.compute_bearing(robot.x');
                    all_classes=horzcat(all_classes,new);
                    length(all_classes);
                else [z(i,:)] = sensor.compute_bearing(robot.x');
                end
                %[z(i,:),new(:,1:3)] = sensor.compute_bearing(robot.x',map,i,numsteps);      %compute the CLASSES
                %all_classes=horzcat(all_classes,new);                                       %at every step! (useful?!)
                %plot(robot.driver.goal(1), robot.driver.goal(2), '*')
                %if nargout == 0
                %if no output arguments then plot each step
                robot.plot_robot();
                drawnow
                %end 
            end
            
            % create the noise for the sensor (why here and not in the sensor class?)
%             a = -0.5;
%             b = 0.5;
%             robot.sensor_noise = (b-a).*rand(map.num,3) + a;
            
            robot.sensor_noise = wgn(map.num,3,0);  % 0 = power of the noise in dB.
            
            add_noise2class(robot.sensor_noise,all_classes,map.num);
            
            % add noise to the bearing reading from the sensor
            z_n = awgn(z,1);
            
            % add noise to the encoder readings (odometry) (old way)
            %robot.x_past_n(:,1) = awgn(robot.x_past(:,1),15);
            %robot.x_past_n(:,2) = awgn(robot.x_past(:,2),15);
            %robot.x_past_n(:,3) = awgn(robot.x_past(:,3),15);
            
            p = robot.x_past;
            hold on
            robot.plot_real('nat');
            
            % Just plot purposes...
            for h=1:map.num
                znew(:,h)=mean(z(:,h));
                z_nnew(:,h)=mean(z_n(:,h));
            end
            
            znew = znew*180/pi;
            z_nnew = z_nnew*180/pi;
            
            x=[];
            figure
            
            for g=1:map.num
                x = [x g];
            end
            
            w1 = .85;
            w2 = .45;
            bar(x,znew,w1,'FaceColor',[0.2 0.2 0.5])
            hold on
            bar(x,z_nnew,w2,'FaceColor',[0 0.7 0.7])
            hold off
            grid on      
            ylabel('Mean Measurement')
            xlabel('Landmark Number')
            legend({'Real Measurement','Noisy Measurement'},'Location','northeast')
            
            diff = zeros(1,map.num);
            for f=1:map.num                
                if (sign(znew(:,f))==-1 && sign(z_nnew(:,f))==-1 ) || (sign(znew(:,f))==1 && sign(z_nnew(:,f))==1 )
                    first = abs(znew(:,f));
                    second = abs(z_nnew(:,f));
                    diff(1,f) = abs(first-second);
                    diff(1,f) = 100-(diff(1,f) * 10);
                else 
                    first = abs(znew(:,f));
                    second = abs(z_nnew(:,f));
                    diff(1,f) = first + second;
                    diff(1,f) = 100-(diff(1,f) * 10);
                end
            end
            
            accuracy = sum(diff)/map.num;
            fprintf('The accuracy of the sensor is %3.1f%%\n',accuracy)
                
                
%             axis([0 100 0 100])
%             hist(znew,20)
%             h = findobj(gca,'Type','patch');
%             set(h,'FaceColor','r','EdgeColor','w','facealpha',0.75)
%             hold on;
%             hist(z_nnew,20)
%             h1 = findobj(gca,'Type','patch');
%             set(h1,'facealpha',0.75);
%             grid on
             
            %xaxis = 1:map.num;
            %plot(xaxis,znew,'r');
            %hold on
            %plot(xaxis,z_nnew,'b');
            %grid
            
        end
        
        function [p,distances] = run_chosen(robot,points)
            % Runs the robot simulation in CHOSEN MODE.
            
            robot.init;
            %robot.clear();
            if ~isempty(robot.driver)
                robot.driver.visualize();
            end
            
            grid
            a=ginput(points);
            robot.visualize();
            
            for h=1:points
                plot(a(h,1),a(h,2),'d','MarkerFaceColor','c');
                hold on
            end
            
            % Compute points distances
            distances=zeros(points,1);
            %numsteps=zeros(points,1);
            for j=1:points
                if j==1
                    distances(j,1)=sqrt((a(j,1)-robot.x(1))^2+(a(j,2)-robot.x(2))^2);
                    %numsteps(j,1)=distances(j,1)*3.5;
                else distances(j,1)=sqrt((a(j,1)-a(j-1,1))^2+(a(j,2)-a(j-1,2))^2);
                    %numsteps(j,1)=distances(j,1)*3.5;
                end
            end
            
            numsteps=compute_numsteps(robot.driver.speed,points,distances);  % timesteps computation depending on speed and distance.
            
            for c=1:points
                %while robot.x(1)~=a(c,1) && robot.x(2)~=a(c,2)
                for i=1:numsteps(c,1)         %numsteps(c,1)
                    robot.onestep_(a(c,:));
                    %if nargout == 0
                    % if no output arguments then plot each step
                    robot.plot_robot();
                    drawnow
                    %end
                    if robot.x(1)==a(c,1) && robot.x(2)==a(c,2)
                        break;
                    end
                end
            end
            
            %             if i < 2
            %                 robot.onestep(a(1,:))
            %                 if nargout == 0
            %                     % if no output arguments then plot each step
            %                     robot.plot_robot();
            %                     drawnow
            %                 end
            %             else
            %                 B=A*round(double(i)/A)==i;
            %                 if B==1 && c<= points
            %                     robot.onestep(a(c,:));
            %                     %plot(robot.driver.goal(1), robot.driver.goal(2), '*')
            %                     if nargout == 0
            %                         % if no output arguments then plot each step
            %                         robot.plot_robot();
            %                         drawnow
            %                         c=c+1;
            %                     end
            %                 end
            %             end
            
            
%             % add noise to the encoder readings (odometry)
%             robot.x_past_n(:,1) = awgn(robot.x_past(:,1),15);
%             robot.x_past_n(:,2) = awgn(robot.x_past(:,2),15);
%             robot.x_past_n(:,3) = awgn(robot.x_past(:,3),15);
            
            p = robot.x_past;
            hold on
            robot.plot_real();
        end
        
        function p = run_sim(robot, T, speed, steer, clip)
            % Runs the vehicle simulation with control inputs chosen by the
            % user. The user can either specify one speed value and one steer
            % value or two values for both. In this case a clip as to be
            % specified as well, representing the portion of the time
            % duration T at which both speed and steer have to switch to
            % the second(s) values provided as inputs.
            %
            % P = robot.run_sim(T, speed, steer) runs the vehicle model for a time T with
            % speed SPEED and steering angle STEER.  P (Nx3) is the path followed and
            % each row is (x,y,theta).
            
            robot.init;
            
            %robot.clear();
            if ~isempty(robot.driver)
                robot.driver.visualize();
            end
            
            robot.visualize();
            if numel(speed)==2 && numel(steer)==2 && ~isempty(clip)
                for i=1:(T/robot.dt)
                    if i>=clip*(T/robot.dt)
                        robot.onestep(speed(2), steer(2));
                        robot.plot(robot.x);            % always plot the robot
                        robot.plot_real();             % just plot the line (robot path)
                        drawnow
                    else
                        %robot.onestep(speed,steer);
                        robot.onestep(speed(1), steer(1));
                        robot.plot(robot.x);            % always plot the robot
                        robot.plot_real();             % just plot the line (robot path)
                        drawnow
                    end
                end
                %robot.plot(robot.x);
            else
                for i=1:(T/robot.dt)
                    robot.onestep(speed,steer);
                    robot.plot(robot.x);               % always plot the robot
                    robot.plot_real();                  % just plot the line (robot path)
                    drawnow
                end
                %robot.plot(robot.x);
            end
            p = robot.x_past;
        end
        
        function h = plot_robot(robot, varargin)
            % Plots vehicle
            % robot.plot_robot(options) plots the robot at a pose given by
            % the current state. If the robot has been previously plotted its
            % pose is updated. The vehicle is as a narrow triangle that has a length robot.dim.
            % robot.plot_robot(x, options) plots the vehicle on the current axes at the pose X.
            
            % OPTIONS
            % 'scale',A    Scales the robot's dimensions wrt axis
            % 'size',B     Robot has length B
            % 'color',C    Color of the robot
            % 'fill'       Filled with color from 'color' option
            
            h = findobj(gcf, 'Tag', 'Robot.plot_robot');
            if isempty(h)
                % no instance of vehicle graphical object found
                h = robot.plot(robot.x, varargin{:});
                set(h, 'Tag', 'Robot.plot_robot');  % tag it
            end
            
            if ~isempty(varargin) && isnumeric(varargin{1})
                Robot.plot(h, varargin{1}); % use passed value
            else
                Robot.plot(h, robot.x);    % use current state
            end
        end
        
        function [out1,out2] = plot_real(robot, varargin)
            % Plots true path followed by the robot.
            if strcmp(varargin,'not')==0
            xyt = robot.x_past;   % the real path is extracted from the path history.
            abc = robot.x_past_n;
            %if nargout == 0
                plot(xyt(:,1), xyt(:,2),'r');
                hold on
                plot(abc(:,1), abc(:,2),'b');
            %else
            %    out1 = xyt;
            %    out2 = abc;
            %end
            else
            xyt = robot.x_past;   % the real path is extracted from the path history.
            %if nargout == 0
                plot(xyt(:,1), xyt(:,2),'r');
            %else
            %    out1 = xyt;
            end
            end   
        %end
        
    end   % methods
    
    methods(Static)
        
        function h_ = plot(x, varargin)
            % Plots robot pose
            % H = robot.plot(x, options) draws a representation of a ground robot as an
            % oriented triangle with pose X (1x3) [x,y,theta].  H is a
            % graphics handle!!
            % If x (nx3) is a matrix it is considered to represent a trajectory in which case
            % the robot graphic is animated.
            % robot.plot(H, x) as above but updates the pose of the graphic represented
            % by the handle H to pose h.
            %
            % OPTIONS
            % 'scale',A    Scales the robot's dimensions wrt axis
            % 'size',B     Robot has length B
            % 'color',C    Color of vehicle.
            % 'fill'       Filled with color from 'color' option
            % 'fps',D      Frames per second in animation mode (default 10)
            
            if isscalar(x) && ishandle(x)
                h = x;
                x = varargin{1};
                x = x(:)';
                T = transl([x(1:2) 0]) * hom_tr_z( x(3) );
                set(h, 'Matrix', T);
                return
            end
            
            option.scale = 1/45;
            option.size = [];
            option.fill = true;
            option.color = 'r';
            option.fps = 10;
            
            [option,args] = tb_optparse(option, varargin);
            
            lines = { 'Color', option.color' };
            if option.fill
                lines = [lines 'fill' option.color ];
            end
            
            % compute the dimensions of the robot
            if ~isempty(option.size)
                d = option.size;
            else
                % get the current axes dimensions
                a = axis;
                d = (a(2)+a(4) - a(1)-a(3)) * option.scale;
            end
            
            % draw the robot!
            points = [d 0;-d -0.7*d;-d 0.7*d]';
            
            h = hgtransform();  %creates a transform graphics object and returns its handle h.
            hp = draw_poly(points, lines{:});
            for hh=hp
                set(hh, 'Parent', h);
            end
            
            if (numel(x) > 3) && (size(x,2) == 3)
                % animation mode
                for i=1:size(x, 1);
                    T = transl([x(i,1:2) 0]) * hom_tr_z( x(i,3) );
                    set(h, 'Matrix', T);
                    pause(1/opt.fps);
                end
            elseif (numel(x) == 3)
                % compute the pose and convert vector form of pose to SE(3)
                x = x(:)';
                T = transl([x(1:2) 0]) * hom_tr_z( x(3) );
                set(h, 'Matrix', T);
            else
                error('bad pose');
            end
            
            if nargout > 0
                h_ = h;
            end
        end
        
        function visualize(~)
            grid on
        end
        
    end % static methods
    
end



