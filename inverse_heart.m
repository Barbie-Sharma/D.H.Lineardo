%Barbie 
%% D.H. Lineardo, heart

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

% Number of points on the heart shape
num_points = 100;

% Define the parameters for the heart shape
t = linspace(0, 2*pi, num_points);
x = 16 * sin(t).^3;
y = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);
scale = 0.05;

% Initialize initial joint angles
q = [pi/3; pi/3];

% Newton-Raphson method to follow the heart shape
for k = 1:num_points
    % Scale and set target coordinates for the heart shape
    target = scale * [x(k), y(k)];
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
