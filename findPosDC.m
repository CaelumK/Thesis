function [dx,new_delta] = findPosDC(A,B,K,x,delta,n,numDrones,numBins)
 %% State Estimation 
%numBins = 3;
x_hat = zeros(n,1);
new_delta = zeros(n,1);
numParams = n/numDrones;

for drone_i = 1:numDrones
    drone_x = x((drone_i-1)*numParams+1:drone_i*numParams); %states for a single drone
    drone_delta = delta((drone_i-1)*numParams+1:drone_i*numParams);
    temp_delta = findDeltaC(drone_x,drone_delta,numBins,numParams); 
    
    for i = 1:size(temp_delta,1)
        partition = temp_delta(i)*(-(numBins-1)/2:(numBins-1)/2); % codewords corresponding to each partition region
        codebook = [0 (partition(1:length(partition)-1)+partition(2:length(partition)))/2 0]; % initial guess of a partition. 
        [~,x_hat((drone_i-1)*numParams+i)] = quantiz(drone_x(i),partition,codebook); % x_hat holds the quantized values of x. 
    end
    
    new_delta((drone_i-1)*numParams+1:drone_i*numParams,1) = temp_delta;
end

%% Get position
w_t = normrnd(0,1,[n,1]);
dx = A*x - 2/5*B*K*x_hat + w_t;

end