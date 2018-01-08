%% Las Project - Alberto Dallolio

% This function generates a noise to be added to the RGB color definition
% for all landmarks. Once the noise is included a cycle evaluates the RGB
% values to establish whether the sensor guess properly the color or not!


function [] = add_noise2class(noise,all_classes,c)

%all_classes_noisy = all_classes(:,10:12);
%all_classes_noisy = awgn(all_classes_noisy,20);
%threshold = 0.5;    %1-0.687;
%threshold2 = 0.387;



all_classes_noisy = all_classes(:,10:12);
all_classes_noisy = all_classes_noisy + noise;
threshold = 0.5;


% for y=1:a   %size(all_classes,1)
%     if all_classes(y,10)==1 && (all_classes_noisy(y,1)>=1-threshold || all_classes_noisy(y,1)<=1+threshold)
%         sprintf('The landmark no. %d is yellow and the sensor sees it as yellow!',y)
%     elseif all_classes(y,11)==1 && all_classes(y,12)==1
%         sprintf('The landmark no. %d is cyan!',y)
%     elseif all_classes(y,11)==0 && all_classes(y,12)==1
%         sprintf('The landmark no. %d is blue!',y)
%     end
% end


for y=1:c
    if all_classes(y,10)==1 && all_classes(y,11)==1
        if all_classes_noisy(y,1)>=1-threshold && all_classes_noisy(y,1)<=1+threshold
            sprintf('The landmark no. %d is yellow and the sensor sees it as yellow!',y)
        else %all_classes_noisy(y,2)>=1-threshold && all_classes_noisy(y,2)<=1+threshold
            sprintf('The landmark no. %d is yellow but the sensor sees it as orange!',y)
        end
    elseif all_classes(y,10)==0
        if all_classes_noisy(y,1)>=0-threshold && all_classes_noisy(y,1)<=0+threshold
            sprintf('The landmark no. %d is green and the sensor sees it as green!',y)
        else sprintf('The landmark no. %d is green but the sensor sees it as orange!',y)
        end
    elseif all_classes(y,11)==0.687
        if all_classes_noisy(y,2)>=0.687-threshold && all_classes_noisy(y,1)<=0.687+threshold
            sprintf('The landmark no. %d is green and the sensor sees it as orange!',y)
        else sprintf('The landmark no. %d is green but the sensor sees it as yellow!',y)
        end
    end
end



end

