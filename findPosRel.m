function [dx,delta] = findPosRel(A,B,K,x,delta,n,~,numBins,y,droneDelta)
 %% State Estimation 
y_hat = zeros(n,1);
 
delta = findDelta(y,delta,numBins,n);

for i = 1:size(delta,1)
    partition = delta(i)*(-(numBins-1)/2:(numBins-1)/2); % codewords corresponding to each partition region
    codebook = [0 (partition(1:length(partition)-1)+partition(2:length(partition)))/5 0]; % initial guess of a partition. 
    [~,y_hat(i)] = quantiz(y(i),partition,codebook); % x_hat holds the quantized values of x. 
end

for i = 1 : length(y_hat)
    if y_hat(i) > 100 
        y_hat(i) = 100; 
    elseif y_hat(i)<-100
        y_hat(i)=-100;
    end
end

%1D
% x_hat = y_hat + [0 0 y_hat(1) 0 y_hat(1)+y_hat(3) 0]';
%3D
% y_hat = y;
x_hat = y_hat + [0 0 0 0 0 0,...
                y_hat(1) y_hat(2) y_hat(3) 0 0 0,...
                y_hat(1)+y_hat(7) y_hat(2)+y_hat(8) y_hat(3)+y_hat(9) 0 0 0]';

%% Get position
w_t = normrnd(0,1,[n,1]);
% dx = A*x - B*K*(x_hat) + w_t;
dx = A*x - B*K*(x_hat-droneDelta) + w_t;

end

