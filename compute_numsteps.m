%% Las Project - Alberto Dallolio

% This function computes the required number of timesteps for plotting
% purposes. Indeed, takes as inputs the speed set by the user in the Path
% object definition, the number of points and the computed distances
% between them. Then, simply applies a multiplicative factor to get the
% number of steps that satisfies the simulation.

%% Implementation

function [ numsteps ] = compute_numsteps(speed,points,distance)
numsteps=zeros(points,1);
for i=1:points
    if speed==1
        numsteps(i,1)=speed*distance(i,1);
    else if speed==2
            numsteps(i,1)=speed*distance(i,1)*2.5;
        else if speed==3
                numsteps(i,1)=speed*distance(i,1)*1.1;
            else if speed==4
                    numsteps(i,1)=speed*distance(i,1)*0.66;
                end
            end
        end
    end
end

end


