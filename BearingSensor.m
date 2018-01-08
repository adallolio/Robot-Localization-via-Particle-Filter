%% Las Project - Alberto Dallolio
% Creates the bearing sensor.
% This class can be connected to Robot and Map classes.

%% PROPERTIES

% - W                   Measurement covariance
% - theta_lim           Bearing limit
% - map                 Map object
% - randstream          Random stream of points

%% METHODS

% - reading                 Bearing observation of random feature
% - compute_bearing         Bearing observation of specific feature
% - BearingSensor           Object constructor
% - compute_loc             Landmark position given robot pose and observation

%% Implementation

classdef BearingSensor < handle
    
    properties
        W                  % measurment covariance
        theta_lim          % bearing limits
        map
        robot
        randstream         % random stream just for Sensors
        num_readings       % number of reading()s
    end
    
    properties (SetAccess = private)
        
    end
    
    methods
        
        function sensor = BearingSensor(robot, map, limit)
            % Creates the constructor
            % S = BearingSensor(robot, map, limit) is an object representing
            % a bearing sensor mounted on the robot object.
            % Observing an environment of known landmarks represented by the
            % map object MAP. The sensor covariance is W (2x2) representing  bearing
            % covariance.
            
            % OPTIONS
            % 'angle', theta                 Detection for angles betwen [-theta +theta]
            % 'angle', [thetamin thetamax]   Detection for angles betwen [thatamin thetamax]
            sensor.robot=robot;
            sensor.map=map;
            sensor.randstream = RandStream.create('mt19937ar');
            theta_range = limit*pi/180;
            sensor.W = [0.1, 1*pi/180].^2;          % in the end I'll use the wgn 
            
            %option.theta_range = [];
            %[option,args] = tb_optparse(option, varargin);
            
            if ~isempty(theta_range)
                if length(theta_range) == 1
                    sensor.theta_lim = [-theta_range theta_range];
                elseif length(theta_range) == 2
                    sensor.theta_lim = theta_range;
                end
            end
            sensor.num_readings = 0;
        end
        
        function k = selectFeature(s)
            k = s.randstream.randi(sensor.map.num);  %randi = uniform discrete distribution
        end
        
        function [z,classes] = compute_bearing(sensor,xr,jf)
            % Computes the bearing bearing.
            % This function also computes the range (distance between the sensor and the landmark)
            % and so the coordinates of the landmark on the planar map.
            % z = sensor.compute_bearing(xr, k) is a sensor observation (1x2), bearing, from robot at
            % pose XR (1x3) to the k'th map feature.
            % z = sensor.compute_bearing(xr) as above but computes bearing to all
            % the landmarks. In z, every feature corresponds to a row.
            % Noise with covariance W is added to each feature (row of z).
            
            if nargin == 2
                xf = sensor.map.map;
            elseif length(jf) == 1
                % s.h(XV, JF)
                xf = sensor.map.map(:,jf);
            else xf=jf;
            end
            % Rough code:
            % dx = xf(1) - xr(1); dy = xf(2) - xr(2);
            % z = atan2(dy, dx) - xr(3);       % bearing measurement
            
            z=zeros(1,sensor.map.num);
            classes=zeros(sensor.map.num,3);
            
            if numel(xf)==2
                xf=xf';
                dx = xf(1,1) - xr(:,1); dy = xf(1,2) - xr(:,2);
                all = [sqrt(dx.^2 + dy.^2) atan2(dy, dx)-xr(:,3) ];   % range/bearing measurement
                z=all(:,2);
                %z = z + sensor.randstream.randn(1) * sensor.W(2);
            else
                for i=1:sensor.map.num
                    if nargout==2  % the computation of the class is carried out only when needed!
                        dx = xf(1,i) - xr(:,1); 
                        dy = xf(2,i) - xr(:,2);
                        all = [sqrt(dx.^2 + dy.^2) atan2(dy, dx)-xr(:,3) ];   % range and bearing measurement (range not needed, but is needed to compute_loc)
                        z(1,i)=all(:,2);                                      % bearing measurements acquisition
                        classes(i,:)=compute_class(sensor.map,i);
                    else
                        dx = xf(1,i) - xr(:,1); dy = xf(2,i) - xr(:,2);
                        all = [sqrt(dx.^2 + dy.^2) atan2(dy, dx)-xr(:,3) ];   % range and bearing measurement (range not needed here, but is needed to compute_loc)
                        z(1,i)=all(:,2);                                      % bearing measurements acquisition
                    end
                    %z(i) = z(i) + sensor.randstream.randn(1) * sensor.W(2);
                end
            end
            
            % add noise with covariance W
            %z = z + sensor.randstream.randn(size(z)) * sqrt(sensor.W);
            %loc = [xr(1)+all(:,1)*cos(z) xr(2)+all(:,1)*sin(z)];
            
            % add noise with covariance W
            
        end
        
        function loc = compute_loc(sensor, xr, k)
            % Computes landmark location
            % P = sensor.compute_loc(xr,z) is the world coordinate (1x2) of a landmark given
            % the sensor observation z (1x2) and vehicle state xr (3x1).
            
            dx = sensor.map.map(1,k) - xr(1,:);
            dy = sensor.map.map(2,k) - xr(2,:);
            all = [sqrt(dx.^2 + dy.^2) atan2(dy, dx)-xr(3,:) ];
            range = all(1);
            bearing = all(2) + xr(3); % bearing angle in robot frame
            
            loc = [xr(1)+range*cos(bearing) xr(2)+range*sin(bearing)];
        end
        
        function [z,jf] = reading(sensor)
            % Gets a random reading.
            % Since the sensor sees all the landmarks always, one reading
            % means the reading of the bearing angles to all landmarks!
            % [z,k] = sensor.reading() is an observation of a random landmark where
            % Z=[theta] is the bearing with additive white Gaussian noise
            % of covariance W (property W). k is the index of
            % the map feature that was observed.
            
            % the sensor that outputs readings at every sampling interval
            sensor.num_readings = sensor.num_readings + 1;
            
            if ~isempty(sensor.theta_lim)
                
                % get the bearing from all landmarks
                z = sensor.compute_bearing(sensor.robot.x');
                jf = 1:size(sensor.map.map,2);
                
                % find all within the range
                k = find(z(1,:) >= sensor.theta_lim(1) & z(1,:) <= sensor.theta_lim(2));
                z = z(:,k);
                jf = jf(k);
                
                if isempty(k)
                    % no landmarks found
                    z = [];
                    jf = NaN;
                elseif length(k) >= 1
                    % more than 1 in range, pick a random one
                    i = sensor.randstream.randi(length(k));
                    z = z(:,i);
                    jf = jf(i);
                end
                
            end
            if ~isempty(z)
                plot(jf);
            end
        end
        
    end
end
