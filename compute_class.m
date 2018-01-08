%% Las Project - Alberto Dallolio
% Computes the class to which the k'th landmark belongs.



function class = compute_class(map,k)

a = isequal(map.colors(k,:),[1 1 0]);
b = isequal(map.colors(k,:),[0 1 0]);
c = isequal(map.colors(k,:),[1.0,0.687,0.387]);

if a==1
    class=[1 1 0];
    return
    %sprintf('The landmark no. %d is yellow!',k)
elseif b==1
    class=[0 1 0];
    return
    %sprintf('The landmark no. %d is green!',k)
elseif c==1
    class=[1.0,0.687,0.387];
    return
    %sprintf('The landmark no. %d is orange!',k)
end

end

