%% Las Project - Alberto Dallolio

% A Map object represents a square 2D environment with a certain number of
% landmarks

%% PROPERTIES

%   - map         Matrix of map feature coordinates 2xN
%   - dim         The dimensions of the map x,y in [-dim,dim]
%   - num         The number of map landmarks

%% METHODS

%   - plot      Plot the feature map
%   - map       Object constructor
%   - features  Gets landmarks from map


%% Implementation

classdef Map < handle
    
    properties
        map         % map features
        dim         % map dimension
        num         % number of features in map
        colors
    end
    
    methods
        
        % Map Constructor
        function map = Map(dim, num)
            % Creates a map of point landmarks
            % map = Map(dim,num) is a Map object that contains num random
            % points in a planar region bounded in [-dim +dim].
            
            dims = dim;
            map.dim = dims;
            map.num = num;
            map.map = dims * (2*rand(2, num)-1);   %are the coordinates of the num landmarks!
            
            map.colors=zeros(map.num,3);
            colorSet=[1 1 0;0 1 0;1.0,0.687,0.387];  % yellow, green and light orange. Dark orange = [1.0,0.4,0.0].  
            
            % blue, cyan and red (no more)
            
            for i=1:map.num
                h=randperm(3);
                %plot(map.map(1,i), map.map(2,i),'Color',colorSet(h(1),:),'Marker','diamond','MarkerFaceColor',colorSet(h(1),:));
                map.colors(i,:)=colorSet(h(1),:);
                hold on                 %the color of each landmark is chosen randomly among the three possible colors.
            end
            
            
        end
        
        function f = feature(map, k)
            % Gets landmarks coordinates from map
            % f = map.feature(k) is the coordinate (2x1) of the k'th map landmark.
            
            if nargin < 3
                f = map.map(:,k);
            else for i=1:(map.num)
                    sprintf('Coordinates of point %0.5g',i)
                    sprintf('%0.5f',map.map(:,i))
                end
            end
        end
        
        function plot(map)
            % Plots the map
            % map.plot() plots the feature map in the current figure, as a square
            % region with dimensions given by the map.dim property.  Each feature
            % is marked by a diamond (shape to be chosen!).
            
            clf                     % clear current figure window
            d = map.dim;
            axis([-d d -d d]);
            xlabel('x');
            ylabel('y');
            
            %map.colors=zeros(map.num,3);
            %colorSet=[0 0 1;0 1 1;1 0 0];           % blue, cyan and red.
            
            
            for i=1:map.num
                %h=randperm(3);
                plot(map.map(1,i), map.map(2,i),'Color',map.colors(i,:),'Marker','diamond','MarkerFaceColor',map.colors(i,:));
                %map.colors(i,:)=colorSet(h(1),:);
                hold on                 %the color of each landmark is chosen randomly among the three possible colors.
            end
            
            legend('Landmarks')
            grid on
                    
%             K = 5; % number of points in each group;
%             M = 3; % number of groups;
%             N = map.num; % total number of points
%             col = repmat(1:M,1); % define colors by index value
%             col = col(:); % now col is Nx1
%             x = rand(N,1);
%             y = rand(N,1);
%             scatter(map.map(1,:)',map.map(2,:)',N,col(:),'d','filled')
%             plot(map.map(1,:)', map.map(2,:)', args{:});
%             plot(x, y, 'filled','p');
%             hold on

        end
        
    end % method
    
end % classdef
