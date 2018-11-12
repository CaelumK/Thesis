function newDelta = findDelta(x,oldDelta,K)

smallDelta = ones(12,1)*0.5;
epsilon = ones(4,1)*0.5;
nu = ones(12,1)*2.8;
L = ones(12,1)*0.875; %droneDist(1.75)/2
lambda = ones(12,1);
%zoom in if delta > search space/2
h = x./(K*oldDelta/2);
Q_bar = zeros(12,1);
if any(abs(h) > 1)
    Q_bar = abs(lambda) + smallDelta;
elseif all(abs(h) <= 1)
    for i = 1:size(Q_bar,1)
       if oldDelta(i) > L(i)
           Q_bar(i) = lambda(i)/(K-nu(i));
       else
           Q_bar(i) = 1; 
       end
    end
end
newDelta = oldDelta.*Q_bar;