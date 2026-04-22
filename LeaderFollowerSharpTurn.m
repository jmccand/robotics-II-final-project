% JSquared - This will perform analysis of leader-follower control
% structure for multi-robot systems

clf; clear; close all;

% Define robot starts (10m apart from each other)
% Since holonomic robots, don't particularly care about rotation
q(:,1,:) = [[0; 0] [-2; -2] [2; -2] [0; -4]];

% Define robot parameters
v_follow = 1;
v_max = 2;
kp = 2;

% Define formation as diamond
q_form(:,:) = [[-2; -2] [4; 0] [-2; -2]];
q_form_1(:,:) = [[-2; -2] [2; -2] [0; -4]];

% Sim properties
dt = 0.1;
k = 1;

% Define curve to follow wrt time
q_goal(:,1) = [0;0];
for i = 1:10000
    q_goal(1,i+1) = q_goal(1,i) - v_follow*dt*cos(atan2(-2*q_goal(1,i)-5,1));
    q_goal(2,i+1) = -q_goal(1,i+1)^2-5*q_goal(1,i+1);
end

x = -(0:0.05:15);
y = -x.^2-5.*x;

% Adjust start position for followers
theta = atan2(q_goal(2,2) - q_goal(2,1), q_goal(1,2) - q_goal(1,1))-pi/2;
rot = [cos(theta) -sin(theta);
       sin(theta) cos(theta)];

for i = 2:length(q(1,1,:))
    q(:,1,i) = rot*q(:,1,i);
end

% Plot robots and path
plot(x,y); hold on; grid on;

for i = 1:4
    robot_plot(i) = scatter(q(1,k,i),q(2,k,i), 'green', 'filled'); 
end
xlim([-50,50]); ylim([-30,50]); xlabel('x (m)'); ylabel('y (m)');
title('Leader-Follower Curve/Sharp Turn')

% Loop for some time
while k < 400
    q(:,k+1,1) = UpdatePos(q(:,k,1),q_goal(:,k),dt,v_max,kp);
    
    if k ~= 1
        theta = atan2(q(2,k+1,1)-q(2,k,1), q(1,k+1,1)-q(1,k,1))-pi/2;
        rot = [cos(theta) -sin(theta);
               sin(theta) cos(theta)];
    end

    for i = 2:length(q(1,1,:))
        q(:,k+1,i) = UpdatePos(q(:,k,i), q(:,k,i-1) + rot*q_form(:,i-1), dt, v_max, kp);

        % Record error
        error(i,k) = norm(q(:,k,1) + rot*q_form_1(:,i-1) - q(:,k,i));
    end

    k = k+1;

    delete(robot_plot);
    % plotting the new position of the robot and its trajectory
    for i = 1:length(q(1,1,:))
        robot_plot(i) = scatter(q(1,k,i),q(2,k,i), 'green', 'filled'); hold on; grid on;
    end

    pause(dt);
end

%% Plot error over time
figure
for i = 2:length(q(1,1,:))
    plot(0:dt:(k-2)*dt,error(i,:)); hold on; grid on;
end
legend("Robot 2", "Robot 3", "Robot 4");
title("Error over time");
ylabel("Error (m)"); xlabel("Time (s)");



%% Functions

% Get the next position of a robot based on current position and goal
function q_next = UpdatePos(qi, qf, dt, v_max, kp)
    error = kp*norm(qf-qi);
    angle = atan2(qf(2)-qi(2),qf(1)-qi(1));

    v = sign(error)*[sign(cos(angle))*min(abs(error*cos(angle)),v_max); sign(sin(angle))*min(abs(error*sin(angle)),v_max)];

    q_next = qi + v*dt;
end