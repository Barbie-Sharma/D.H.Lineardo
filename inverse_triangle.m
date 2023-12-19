%% D.H. Lineardo, triangle

%Barbie 
%Simulation of a 2-axis planar RR robot

clc 
clearvars
clf

l1 = 1;
l2 = 1;

%DH parameters 

%link 1
a1 = l1;
alpha1 = 0;
d1 = 0;

%link 2
a2 = l2;
alpha2 = 0;
d2 = 0;

% Vertices of a triangle
vertices = [0, 0; 2, 4; -3, 2]; % Define vertices here

% Number of points for each segment of the triangle
num_points = 100;

% Initialize initial joint angles
q = [pi/3; pi/3];

% Simulate movement along the triangle's vertices
for i = 1:size(vertices, 1)
    start_point = vertices(i, :);
    end_point = vertices(mod(i, size(vertices, 1)) + 1, :);
    
    % Define a trajectory between start and end points
    x_traj = linspace(start_point(1), end_point(1), num_points);
    y_traj = linspace(start_point(2), end_point(2), num_points);

    % Newton-Raphson method to follow the trajectory
    for k = 1:num_points
        target = [x_traj(k), y_traj(k)];
        mu_a = target.';
        theta1 = q(1);
        theta2 = q(2);

        % Jacobian matrix
        J = [-a2*sin(theta1 + theta2) - a1*sin(theta1), -a2*sin(theta1 + theta2);
              a2*cos(theta1 + theta2) + a1*cos(theta1),  a2*cos(theta1 + theta2);];

        % Estimated task space position
        mu_e = [a1*cos(theta1) + a2*cos(theta1 + theta2);
                a1*sin(theta1) + a2*sin(theta1 + theta2)];

        % Error (task-space error)
        delta = mu_a - mu_e;

        % Tolerance check 
        if norm(delta) < 1e-5
            break;
        end

        % Revised joint space positions
        q = q + pinv(J) * delta;

        % Plotting the values
        O1x = a1*cos(theta1);
        O1y = a1*sin(theta1);

        O2x = mu_e(1);
        O2y = mu_e(2);

        link1 = line([0, O1x], [0, O1y]);
        link2 = line([O1x, O2x], [O1y, O2y]);

        set(link1, 'LineWidth', 2);
        set(link2, 'LineWidth', 5);

        hold on 

        plot(O2x, O2y, 'g.', 'MarkerSize', 10);
        plot(0, 0, 'ks', 'MarkerSize', 10)

        axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)]);

        grid on
        pause(0.1);
        delete(link1);
        delete(link2);
    end
end
