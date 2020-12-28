global m1 d1 l1 Ic1zz m2 d2 l2 g0 Fa
global Mm Dm Km

% Robot parameters
d1 = 0.5;   % [m]
d2 = 0.5;   % [m]
l1 = 1;     % [m]
l2 = 1;     % [m]

m1 = 1;     % [kg]
m2 = 1;     % [kg]

Ic1zz = 10; % [kg m^2]
Ic2zz = 10; % [kg m^2]

g0 = 9.81;

% Initial Condition
q0 = [pi/4; pi/4];
dq0 = [0;0];

% Time parameters
Tend = 20;
dt = 0.1;
%samples = 1001;
samples = Tend/dt;
timeSpace = linspace(0, Tend, samples);

% Plot parameters
xlimmin = -2;
xlimmax = 2;
ylimmin = -2.5;
ylimmax = 2.5;

% Linear trajectory
q1TrajDes  = qd(1) + (timeSpace-timeSpace(1))/(timeSpace(end)-timeSpace(1))*(qd(1)-0.5-qd(1));
q2TrajDes  = qd(2) + (timeSpace-timeSpace(1))/(timeSpace(end)-timeSpace(1))*(qd(2)-0.5-qd(2));

% Sin traj
%q1TrajDes = q0(1)+0.0.*sin(timeSpace);
%q1TrajDes = q0(1)+sin(timeSpace);
%q2TrajDes = q0(2)+cos(timeSpace)-cos(0.0);

% Constant
%q1TrajDes = q0(1) + 0.5 + zeros(size(timeSpace));
%q2TrajDes = q0(2) - 0.5 + zeros(size(timeSpace));
%dq1TrajDes = zeros(size(timeSpace));
%dq2TrajDes = zeros(size(timeSpace));

% Plot the trajectories
figure;
subplot(2,1,1)
plot(timeSpace, q1TrajDes); hold on; grid on;
subplot(2,1,2)
plot(timeSpace, q2TrajDes); hold on; grid on;

% Plot initial configuration
figure;
title(num2str(timeSpace(1)));
hold on; grid on; axis equal;
axis equal;
xlim([xlimmin xlimmax]);
ylim([ylimmin ylimmax]);

% Plot the robot
p0 = directKinematics2R_details(q0,l1,l2);
p1 = p0(1:2);
pEE = p0(3:4);
plot([0,p1(1)],[0, p1(2)], 'b');
hold on; grid on;
plot([p1(1), pEE(1)],[p1(2), pEE(2)], 'r');

% Plot the trajectories
pTraj = zeros(4, samples);
for i = 1 : 1 : samples
    qToTraj = [q1TrajDes(i); q2TrajDes(i)];
    pCurr = directKinematics2R_details(qToTraj,l1,l2);
    pTraj(:,i) = pCurr;
end

pTraj_p1 = pTraj(1:2,:);
pTraj_pEE = pTraj(3:4,:);
plot(pTraj_p1(1,:), pTraj_p1(2,:), 'b'); hold on;
plot(pTraj_pEE(1,:), pTraj_pEE(2,:), 'r');

% No forces
Fa = [0.0;0.0];

% Last configuration for the first configuretion? q0
q_old = q0;
dq_old = dq0;

% Gains of the PD control
Kp = 40.*eye(2,2);
Kd = 70.*eye(2,2);

% Structure for saving data
qStack = q0;
qdStack = [q1TrajDes(1); q2TrajDes(1)];
tStack = timeSpace(1);
dqStack = dq0;

% Time loop t(i)
for i = 1 : 1 : samples
    
    dt = timeSpace(2) - timeSpace(1);
    
    % Desired current reference for the joint
    qd = [q1TrajDes(i); q2TrajDes(i)];
    dqd = [dq1TrajDes(i); dq2TrajDes(i)];
    
    % Sensor information
    q_sensors = q_old; %+ 0.01.*rand(2,1);
    dq_sensors = dq_old;
    
    % Control law
    g = gravityTerm2R(qd);
    u = Kp * (qd - q_sensors) + Kp * (dqd - dq_sensors) + g;
    
    % Dynamic model of the robot
    ddq = dynamicModel2RMATLAB(u,q_sensors,dq_sensors,Fa);
    dq_new = dq_old + dt * ddq;
    q_new = q_old + dt * dq_new;
    
    q_old = q_new;
    dq_old = dq_new;
    
    % Visualisation
    pToPlot = directKinematics2R_details(q_new,l1,l2);
    p1 = pToPlot(1:2);
    pEE = pToPlot(3:4);
    
    % Plot the robot
    plot([0,p1(1)],[0, p1(2)], 'b'); hold on;
    plot([p1(1), pEE(1)],[p1(2), pEE(2)], 'r');
    
    % Plot the desired trajectories
    plot(pTraj_p1(1,:), pTraj_p1(2,:), 'b'); hold on;
    plot(pTraj_pEE(1,:), pTraj_pEE(2,:), 'r');
    title(num2str(timeSpace(i)));
    
    % Plot final desired trajectory points
    plot(pTraj_p1(1,end), pTraj_p1(2,end), '*b'); hold on;
    plot(pTraj_pEE(1,end), pTraj_pEE(2,end), '*r');
    
    % Limit the axes of the plot
    axis equal;
    xlim([xlimmin xlimmax]);
    ylim([ylimmin ylimmax]);
    
    drawnow;
    hold off;
    
    % Increase the stack
    qStack = [qStack q_old];
    dqStack = [dqStack dq_old];
    tStack = [tStack timeSpace(i)];
    qdStack = [qdStack qd];
end

% Plot the error
figure;
subplot(3,1,1)
plot(tStack, qStack(1,:), 'r'); hold on; grid on;
plot(tStack, qStack(2,:), 'b'); hold on; grid on;
title('q actual position');
legend('q1','q2');
subplot(3,1,2)
plot(tStack, qdStack(1,:), 'r'); hold on; grid on;
plot(tStack, qdStack(2,:), 'b'); hold on; grid on;
title('q desired position');
legend('q1','q2');
subplot(3,1,3)
plot(tStack, qdStack(1,:)- qStack(1,:), 'r'); hold on; grid on;
plot(tStack, qdStack(2,:)- qStack(2,:), 'b'); hold on; grid on;
title('error plot');
legend('error q1','error q2');

for i = 1 : 1 : samples
    
    % Extract the configuration to plot
    qToPlot = [q1TrajDes(i); q2TrajDes(i)];
    
    % Compute the link positions
    pToPlot = directKinematics2R_details(qToPlot,l1,l2);
    p1 = pToPlot(1:2);
    pEE = pToPlot(3:4);
    
    % Plot the robot
    plot([0,p1(1)],[0, p1(2)], 'b'); hold on;
    plot([p1(1), pEE(1)],[p1(2), pEE(2)], 'r');
    
    % Limit the axes
    axis equal;
    xlim([xlimmin xlimmax]);
    ylim([ylimmin ylimmax]);
    
    % Plot the trajectories  
    plot(pTraj_p1(1,:), pTraj_p1(2,:), 'b'); hold on;
    plot(pTraj_pEE(1,:), pTraj_pEE(2,:), 'r');
    
    % Draw and delete hold
    drawnow;
    hold off;
    if i == 1
        pause;
    end
end

function p0 = directKinematics2R_details(q0,l1,l2)
    % First two elements are x & y of first link
    % Second two elements are x & y of end effector
    p0 = [(l1*cos(q0(1))),(l1*sin(q0(1))),(l1*cos(q0(1))+l2*cos(q0(1)+q0(2))),(l1*sin(q0(1))+l2*sin(q0(1)+q0(2)))];
end

function g = gravityTerm2R(qd)

    g = sin((qd(1)+qd(2)));
end

function ddq = dynamicModel2RMATLAB(u,q_sensors,dq_sensors,Fa)

    ddq = (u + (q_sensors - dq_sensors) + (Fa(1)-Fa(2))) * 0.2;
end