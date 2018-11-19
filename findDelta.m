function newDelta = findDelta(x,oldDelta,numBins,n)

%% Preliminaries 

% Amount to increase
smallDelta = ones(n,1)*3;

% Rate of decrease
nu = ones(n,1)*(numBins-1.9); 

% Minimum bin size
L = ones(n,1)*2.5; %droneDist(10)/2

% Spectum of A matrix
lambda = ones(n,1);

% Ratio factor for each state in X
h = 2*x./(numBins*oldDelta);

% Update factor
Q_bar = zeros(n,1);


if any(abs(h) > 1)
    Q_bar = abs(lambda) + smallDelta;
    
elseif all(abs(h) <= 1)
    for i = 1:size(Q_bar,1)
       if oldDelta(i) > L(i)
           Q_bar(i) = lambda(i)/(numBins-nu(i));
       else
           Q_bar(i) = 1; 
       end
    end
end

newDelta = oldDelta.*Q_bar;