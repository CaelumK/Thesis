%% Simplified model for control of underwater drones
clear all; 
close all; 

%% Preliminaries
Ts = 1;
tspan = 2:1:100;
numDrones = 3; % Number of drones

%% Initializing x0 and desired final locations
x0 = normrnd(0, 100, [1,12]);
final_positions = [-10, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0];

%% Getting state space and generating optimal control
[A,B,C,D] = twoDStateSpace();
Q = C'*C;
R = eye(2*numDrones);
[K,S,e] = lqrd(A,B,Q,R,Ts);

delta_0 = 20*ones(12,1);

[dx, delta] = findPos(A,B,K,x0',delta_0,12);

x = [x0 ; dx'];

%% System

for i = tspan % run though the time span
    [dx, delta] = findPos(A,B,K,x(end,:)',delta,12);
    x = [x ; dx' - final_positions];    
end


figure();
plot(0:1:100,x(:,1))
hold on
plot(0:1:100,x(:,5))
hold on
plot(0:1:100,x(:,9))
legend('Position of drone 1', 'Position of drone 2','Position of drone 3')
