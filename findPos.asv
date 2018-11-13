function [dx,delta] = findPos(A,B,K,x,delta)
 %% State Estimation 

numBins = 8;
x_hat = zeros(12,1);
delta = findDelta(x,delta,numBins);
for i = 1:size(delta,1)
    partition = delta(i)*(-numBins/2:numBins/2); % codewords corresponding to each partition region
    codebook = [-numBins/2 (partition(1:length(partition)-1)+partition(2:length(partition)))/2 numBins/2]; % initial guess of a partition. 
    [~,x_hat(i)] = quantiz(x(i),partition,codebook); % quants holds the quantized values of x. 
end
%% Get position
w_t = randn(12,1);
dx = A*x - B*K*x + w_t;
% dx = (A-B*K)*x;

end