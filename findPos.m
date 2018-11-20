function [dx,new_delta] = findPos(A,B,K,x,delta,n)
 %% State Estimation 
numBins = 18; % Including positive and negative overflow bin (2 bins for overflow!)

delta = findDelta(x,delta(1),numBins);

partition = delta*(-(numBins-2)/2:(numBins-2)/2); % codewords corresponding to each partition region
codebook = [0 (partition(1:length(partition)-1)+partition(2:length(partition)))/2 0]; % initial guess of a partition. 
[~,x_hat] = quantiz(x,partition,codebook); % x_hat holds the quantized values of x. 

new_delta = delta;

%% Get position
w_t = normrnd(0,1,[n,1]);
dx = A*x - B*K*x_hat' + w_t;

end