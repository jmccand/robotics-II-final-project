% Jaiden Lemm - Homework 2 Q3 - This code will impliment the potential
% field algorithm for the given environment and plot the potential field
% strength

%% Initial Variable Setup

clear; clf; close all;

% Start and Goal Location
q_start = [-40; -30];
q_goal = [20; 40];

% Define robot starts (in formation)
% Since holonomic robots, don't particularly care about rotation
q(:,1,:) = [[0; 0] [-2; -2] [2; -2] [0; -4]];
for i = 1:length(q(1,1,:))
    q(:,1,i) = q(:,1,i) + q_start;
end

% Define formation as diamond
q_form(:,:) = [[-2; -2] [4; 0] [-2; -2]];
q_form_1(:,:) = [[-2; -2] [2; -2] [0; -4]];

% Sim properties
k = 1;
dt = 0.1; 

% Robot properties
v_max = 2;

kp = 2;
xi = 1;
eta = 1000;
q_star = 3;

% Obstacle definition
circle = [-10, -10, 10]; % Circle center , radius
rect = [0; 10; 20; 20]; % Rectangle start, rectangle end

% Plot Plot initial conditions and obstacles;
for i = 1:4
    robot_plot(i) = scatter(q(1,k,i),q(2,k,i), 'green', 'filled'); hold on; grid on;
end

plot(q(1,:,1),q(2,:,1), "Marker", ".", "Color", "b", "MarkerSize",50); hold on; grid on;
plot(q_goal(1),q_goal(2), "Marker", "*", "Color", "g", "MarkerSize",30,"LineWidth",3);
rectangle('Position', [circle(1:2)-circle(3), [2 2]*circle(3)], 'Curvature', [1 1], "FaceColor","black");
rectangle('Position', [rect(1:2); rect(3)-rect(1); rect(4)-rect(2)], 'FaceColor',"black");
xlim([-50 50]); ylim([-50 50])
title("Jaiden Lemm - HW2 Q3 - Path in Environment");
xlabel("X pos (m)"); ylabel("Y pos (m)");


%% Potential field algo
while norm(q(:,k,1)-q_goal) > 0.1
    x = q(1,k,1);
    y = q(2,k,1);

    % Attractive field
    U_att = -xi.*(q(:,k,1) - q_goal);

    % Limit to max velocity
    if norm(U_att) > v_max
        U_att = U_att./norm(U_att)*v_max;
    end
    
    % Find min distance between robot and objects

    U_rep = U_rep_q(q(:,k,1), circle, rect, eta, q_star);

    % Next point
    q(:,k+1,1) = q(:,k,1) + (U_att + U_rep)*dt;

    theta = atan2(q(2,k+1,1)-q(2,k,1), q(1,k+1,1)-q(1,k,1))-pi/2;
    rot = [cos(theta) -sin(theta);
           sin(theta) cos(theta)];

    for i = 2:length(q(1,1,:))
        U_rep = U_rep_q(q(:,k,i), circle, rect, eta, q_star);

        q(:,k+1,i) = UpdatePos(q(:,k,i), q(:,k,i-1) + rot*q_form(:,i-1), dt, v_max, kp) + U_rep*dt;  

        % Record error
        error(i,k) = norm(q(:,k,1) + rot*q_form_1(:,i-1) - q(:,k,i));
    end

    k = k+1;

    delete(robot_plot)
    for i = 1:length(q(1,1,:))
        robot_plot(i) = scatter(q(1,k,i),q(2,k,i), 'green', 'filled');
    end

    pause(dt)
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

% This function calculates the distance to the circle and the closest point
% on the circle
% Inputs:   q as current position
%           circle as array of circle center and radius
% Output:   dist as distance to circle
%           q_obs as closest point on circle
function [dist, q_obs] = circDist(q, circle)
    dist = norm(q-circle(1:2)');
    if dist > circle(3)
        dist = dist - circle(3);
        q_obs = circle(1:2)' + (q - circle(1:2)')./norm(circle(1:2)' - q)*circle(3);
    else
        dist = 0;
        q_obs = q;
    end
end


% This function calculates the distance to the rectangle and the closest 
% point on the rectangle
% Inputs:   q as current position
%           rect as array of bottom left to top right rectangle corner
% Output:   dist as distance to rectangle
%           q_obs as closest point on retangle
function [dist, q_obs] = rectDist(q, rect)
    if (q(1) <= rect(1) && q(2) >= rect(4)) % Top left
        dist = norm(q - [rect(1); rect(4)]);
        q_obs = [rect(1); rect(4)];
    
    elseif q(1) > rect(1) && q(1) < rect(3) && q(2) > rect(4) % Top
        dist = abs(q(2) - rect(4));
        q_obs = [q(1); rect(4)];

    elseif q(1) >= rect(3) && q(2) >= rect(4) % Top right
        dist = norm(q - [rect(3); rect(4)]);
        q_obs = [rect(3); rect(4)];

    elseif (q(2) > rect(2) && q(2) < rect(4)) && q(1) > rect(3) % Right
        dist = abs(rect(3) - q(1)); 
        q_obs = [rect(3); q(2)];

    elseif (q(1) >= rect(3) && q(2) <= rect(2)) % Bottom right
        dist = norm(q - [rect(3); rect(2)]);
        q_obs = [rect(3); rect(2)];

    elseif (q(1) > rect(1) && q(1) < rect(3)) && q(2) < rect(2) % Bottom
        dist = abs(rect(2) - q(2)); 
        q_obs = [q(1); rect(2)];

    elseif q(1) <= rect(1) && q(2) <= rect(2) % Bottom left
        dist = norm(q - [rect(1); rect(2)]);
        q_obs = [rect(1); rect(2)];

    elseif (q(2) > rect(2) && q(2) < rect(4)) && q(1) < rect(1) % Left
        dist = abs(rect(1) - q(1));
        q_obs = [rect(1); q(2)];

    else % In the obstacle
        dist = 0;
        q_obs = q;
    end
end


% This function calculates the repulsive field for a given obstacle
% Inputs:   q as current position
%           dist as distance to obstacle
%           q_obs as obstacle point closest to robot
%           eta as repulsive field constant
%           q_star as repulsive field distance
% Output:   U_rep as repulsive field
function U_rep = U_rep_obs(q, dist, q_obs, eta, q_star)
    U_rep = eta/2*(1/q_star - 1/dist)/dist^2.*(q-q_obs)./norm(q - q_obs);
end


function U_rep = U_rep_q(q, circle, rect, eta, q_star)
    % Circle distance and closest point
    [dist_c, q_obsc] = circDist(q,circle);
    
    % Rectangle distance and closest point
    [dist_r, q_obsr] = rectDist(q,rect);

    % Repulsive field due to each obstacle
    U_rep = 0;
    
    if dist_c <= q_star
        U_rep = U_rep - U_rep_obs(q, dist_c, q_obsc, eta, q_star);
    end
        
    if dist_r <= q_star
        U_rep = U_rep - U_rep_obs(q, dist_r, q_obsr, eta, q_star);
    end
end

% Get the next position of a robot based on current position and goal
function q_next = UpdatePos(qi, qf, dt, v_max, kp)
    error = kp*norm(qf-qi);
    angle = atan2(qf(2)-qi(2),qf(1)-qi(1));

    v = sign(error)*[sign(cos(angle))*min(abs(error*cos(angle)),v_max); sign(sin(angle))*min(abs(error*sin(angle)),v_max)];

    q_next = qi + v*dt;
end