function [dx,new_delta] = findPos(A,B,K,x,delta,n)
 %% State Estimation 

numBins = 4;
x_hat = zeros(n,1);
new_delta = [];
for drone_i = 1:3
    drone_x = x((drone_i-1)*4+1:drone_i*4);
    temp_delta = findDelta(drone_x,delta((drone_i-1)*4+1:drone_i*4),numBins,4);

for i = 1:size(temp_delta,1)
    partition = temp_delta(i)*(-numBins/2:numBins/2); % codewords corresponding to each partition region
    codebook = [-temp_delta(i)*numBins/2 (partition(1:length(partition)-1)+partition(2:length(partition)))/2 delta(i)*numBins/2]; % initial guess of a partition. 
    [~,x_hat((drone_i-1)*4+1:drone_i*4)] = quantiz(drone_x(i),partition,codebook); % x_hat holds the quantized values of x. 
end
new_delta = [new_delta;temp_delta];
end

%% Get position
w_t = normrnd(0,1,[n,1]);
dx = A*x - B*K*x_hat + w_t;

end