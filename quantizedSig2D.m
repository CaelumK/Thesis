%% MTHE 493: Stability for the Idealized Case - Two-Dimensional Case
% This script 
clear all; 
close all; 

%% Initializing model parameters 
tspan = 0:0.1:10; % Time to stabilization
numDrones = 3; % Number of drones
numParam = 4; % Number of parameters
n = numParam*numDrones; % Number of states
x0 = [20 0 6 12 5 0 5 2 6 0 5 2]'; % Initial state [P1x P1y V1x V1y P2x P2y V2x V2y P3x P3y V3x V3y] 
droneRad = 1; % metres 
searchArea = [100 1]; % metres 
overlap = 0.25; % metres 
subArea= numDrones*2*droneRad-overlap*(numDrones-1); %section of the search area the drones will cover at a time
droneDist = 2*droneRad - overlap; %distance from center of one drone to the center of the next drone

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
[A,B,C,D] = getStateSpace;

 %% Check Controllability 
 if rank(ctrb(A,B)) == rank(A) 
     fprintf('System is controllable')
 else 
     fprintf('System is not controllable')
     return
 end

 %% LQR Control Implementation 
 ICE = eye(12);
 Vn = .001; 
 DICE = zeros(12,5);
 DICE = [DICE [Vn Vn Vn Vn Vn Vn Vn Vn Vn Vn Vn Vn]'];
 Q = C'*C;
 R = eye(2*numDrones); % 2 because 2 dimensional
 [K,~,~] = lqr(A,B,Q,R);
 delta_0 = ones(12,1)*8;% something in the overflow region
 delta = delta_0;
 x = zeros(12,length(tspan));
 x(:,1) = x0;
 for i = 2:length(tspan)
     [x(:,i),delta] = findPos(A, B, K, x(:,i-1), delta);
 end
 x = x';
 
 t = tspan;
 %[t,x] = ode45(@(t,x)findPos(A,B,-K,x-droneDelta,delta,subArea),tspan,x0);
 y = zeros(size(t,1),n-numParam);
 
for time_i = 1:size(t,1)
    y(time_i,:) = (C*x(time_i,:)')';
end

figure();
plot(t,x(:,1))
hold on
plot(t,x(:,5))
hold on
plot(t,x(:,9))
legend('Position of drone 1', 'Position of drone 2','Position of drone 3')

%  %% Simulation showing position changes 
%  simFig = figure();
%  axis([0 subArea -1 1]);
%  for time_i = 1:size(t,1)
%     plot(x(time_i,1),x(time_i,2),'o',x(time_i,5),x(time_i,6),'o',x(time_i,9),x(time_i,10),'o');
%     axis([0 subArea -1 1]);
%     drawnow
%  end