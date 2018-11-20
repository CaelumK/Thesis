function newDelta = findDeltaC(x,oldDelta,numBins)
%% Preliminaries 
% Amount to increase
smallDelta = 3;

% Rate of decrease
nu = numBins-1.9; 

% Minimum bin size
L = 2.5; 

% Spectrum of A matrix
lambda = 1;

% Ratio factor for each state in X
h = 2*x./(numBins*oldDelta);

% Update factor
Q_bar = 0;

if any(abs(h) > 1)
    Q_bar = abs(lambda) + smallDelta;
    
elseif all(abs(h) <= 1)
       if oldDelta > L
           Q_bar = lambda/(numBins-nu);
       else
           Q_bar = 1; 
       end
end
newDelta = oldDelta*Q_bar;
end
