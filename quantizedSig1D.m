%% MTHE 493: Stability for the Idealized Case - Two-Dimensional Case
clear all; 
close all; 

%% Initializing model parameters 
t_sample = 1;
tspan = 0:t_sample:10000; % Time to stabilization
numDrones = 3; % Number of drones
numParam = 2; % Number of parameters
n = numParam*numDrones; % Number of states
searchArea = [100 1]; % metres 
x0 = normrnd(0, searchArea(1), [n,1]); % Initial state [P1x P1y V1x V1y P2x P2y V2x V2y P3x P3y V3x V3y] 
x0([2,4,6],1) = zeros(3,1);
droneRad = 5; % sensor radius in metres 
overlap = 0.5; % metres 
droneDist = 2*droneRad - overlap; %distance from center of one drone to the center of the next drone

%% Determine final position of drones
xPos = zeros(numDrones,1); %array that contains just the positions in the x direction
count = 1; 

%gets the initial position values in the x direction and puts them in xPos array
for i = 1:numParam:size(x0) 
    xPos(count,1) = x0(i);
    count = count + 1;
end

xPosSort = sort(xPos); 
droneDelta = zeros(n,1); %array that tells us the desired positions of the drones

%use the sorted xPos array to determine which order the drones should be in
%depending on their initial positions (they will go to the position closest
%to them)
for i = 1:size(xPosSort)
   index = find(xPosSort == xPos(i));
   droneDelta(numParam*i-numParam+1,1) = droneRad + (index-1)*droneDist;
end

%% Set-up - not generalized yet
% Defining state- space matrices A, B, C, and D for 3 drones. 
[A,B,C,D] = oneDStateSpace();

 %% LQR Control Implementation 
 Q = C'*C;
 R = eye(numDrones); % 2 because 2 dimensional
 [K,~,~] = lqrd(A,B,Q,R,t_sample);
 
 numBins = [3];
 EV_x = zeros(n,length(numBins)); %expected value
 
 for j = 1 : length(numBins)
 delta = ones(n,1)*20; % something in the overflow
 
 % Initializing signals x and y
 x = zeros(n,length(tspan));
 x(:,1) = x0;
 y = zeros(n-numParam,size(tspan,2));
 % Find our x
 for i = 2:length(tspan)
     [dx,delta] = findPosDC(A, B, K, x(:,i-1), delta, n, numDrones, numBins(j));
     x(:,i) = dx - droneDelta;
%      y(:,i) = (C*x(:,i))';
 end
 
 x = x';
 y = y';
 
 EV_x(:,j) = (sum(x.^2,1)/length(tspan));
 
%% Plotting
figure();
plot(tspan,x(:,1))
hold on
plot(tspan,x(:,3))
hold on
plot(tspan,x(:,5))
legend('Position of drone 1', 'Position of drone 2','Position of drone 3')
 end
%  figure()
%  plot(numBins,EV_x(1,:))
%  hold on 
%  plot(numBins,EV_x(3,:))
%  hold on
%  plot(numBins,EV_x(5,:))
