%% Las Project - Alberto Dallolio
% Computes a homogeneous transformation matrix of a rotation around z-axis.

function T = hom_tr_z(theta,deg)
            
            if nargin > 1 && strcmp(deg, 'deg')
                theta = theta *pi/180;
            end
            
            ct = cos(theta);
            st = sin(theta);
            R = [
                ct  -st  0
                st   ct  0
                0    0   1
                ];
            
            T = [R [0 0 0]'; 0 0 0 1];
        end