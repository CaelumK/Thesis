function [dx,delta] = findPos(A,B,K,x,delta,n)
 %% State Estimation 

numBins = 64;
x_hat = zeros(n,1);
delta = findDelta(x,delta,numBins,n);

for i = 1:size(delta,1)
    partition = delta(i)*(-numBins/2:numBins/2); % codewords corresponding to each partition region
    codebook = [-numBins/2 (partition(1:length(partition)-1)+partition(2:length(partition)))/2 numBins/2]; % initial guess of a partition. 
    [~,x_hat(i)] = quantiz(x(i),partition,codebook); % quants holds the quantized values of x. 
end
%% Get position
w_t = normrnd(0,1,[n,1]);
dx = A*x - B*K*x_hat + w_t;

end