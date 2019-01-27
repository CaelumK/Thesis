function newDelta = findDelta(x,oldDelta,numBins,n)
%% Preliminaries 
% Amount to increase
smallDelta = ones(n,1)*1.01;

% Rate of decrease
nu = ones(n,1)*1.01; 

% Minimum bin size
L = ones(n,1)*2.5; %droneDist(10)/2

% Spectum of A matrix
lambda = ones(n,1);

% Ratio factor for each state in X
h = 2*x./(numBins*oldDelta);

% Update factor
Q_bar = zeros(n,1);

if any(abs(h) > 1)
    h_pos = find(abs(h) > 1);
    Q_bar(h_pos) = abs(lambda(h_pos)) + smallDelta(h_pos);
end
if any(abs(h) <= 1)  
    h_neg = find(abs(h) <= 1);
    for i = 1:size(Q_bar(h_neg),1)
       if oldDelta(h_neg(i)) > L(h_neg(i))
           Q_bar(h_neg(i)) = lambda(h_neg(i))/(nu(h_neg(i)));
       else
           Q_bar(h_neg(i)) = 1; 
       end
    end
end

newDelta = oldDelta.*Q_bar;