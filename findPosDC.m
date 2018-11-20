function [dx,new_delta] = findPosDC(A,B,K,x,delta,n,numDrones)
 %% State Estimation 

numBins = 6;
x_hat = zeros(n,1);
new_delta = zeros(n,1);
numParams = n/numDrones;

for drone_i = 1:numDrones
    drone_x = x((drone_i-1)*numParams+1:drone_i*numParams); %states for a single drone
    drone_delta = delta((drone_i-1)*numParams+1:drone_i*numParams);
    temp_delta = findDelta(drone_x,drone_delta(1),numBins,1); %only need to give it one delta because the delta for all the states of the drone should be the same

    partition = temp_delta*(-(numBins-2)/2:(numBins-2)/2); % codewords corresponding to each partition region
    codebook = [0 (partition(1:length(partition)-1)+partition(2:length(partition)))/2 0]; % initial guess of a partition. 
    [~,x_hat((drone_i-1)*numParams+1:drone_i*numParams)] = quantiz(drone_x,partition,codebook); % x_hat holds the quantized values of x. 

    new_delta((drone_i-1)*numParams+1:drone_i*numParams,1) = temp_delta;
end

%% Get position
w_t = normrnd(0,1,[n,1]);
dx = A*x - B*K*x_hat + w_t;

end