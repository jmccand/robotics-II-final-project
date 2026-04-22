% JSquared - This will perform analysis of leader-follower control
% structure for multi-robot systems

clf; clear; close all;

% Define robot starts
% Since holonomic robots, don't particularly care about rotation
q(:,1,:) = [[0; 0] [-2; -2] [2; -2] [0; -4]];

q_goal = [0; 40];

% Define formation as diamond
q_form(:,:) = [[-2; -2] [4; 0] [-2; -2]];
q_form_1(:,:) = [[-2; -2] [2; -2] [0; -4]];

% Define robot parameters
v_max = 2;
kp = 2;
ki = 0.01;

% Sim properties
dt = 0.1;
k = 1;

for i = 1:4
    robot_plot(i) = scatter(q(1,k,i),q(2,k,i), 'green', 'filled'); hold on; grid on;
end
xlim([-50,50]); ylim([-30,50]); xlabel('x (m)'); ylabel('y (m)');
title('Leader-Follower Straight Line')

% Loop for some amount of k
while norm(q(:,k,1) - q_goal) > 0.5
    q(:,k+1,1) = UpdatePos(q(:,k,1),q_goal,dt,1,kp);
    
    for i = 2:length(q(1,1,:))
        q(:,k+1,i) = UpdatePos(q(:,k,i), q(:,k,i-1) + q_form(:,i-1), dt, v_max, kp);

        % Record error
        error(i,k) = norm(q(:,k,1) + q_form_1(:,i-1) - q(:,k,i));
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
function q_next = UpdatePos(qi,qf,dt,v_max,kp)
    error = kp*norm(qf-qi);
    angle = atan2(qf(2)-qi(2),qf(1)-qi(1));

    v = sign(error)*[sign(cos(angle))*min(abs(error*cos(angle)),v_max); sign(sin(angle))*min(abs(error*sin(angle)),v_max)];

    q_next = qi + v*dt;
end