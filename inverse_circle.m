%% D.H. Lineardo, circle

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

%radius and number of points on circle
radius = 1;
num_points = 100;

%angles around circle
angles = linspace(0, 2*pi, num_points);

% Calculate target coordinates for each angle
target = [radius * cos(angles); radius * sin(angles)];

%inital guess 
q = [pi/3; pi/3];

%newton-rapson method
for k = 1:num_points
    mu_a = target(:, k);
    theta1 = q(1);
    theta2 = q(2);

    %Jacobian matrix (partial derivative, derived)
    J = [-a2*sin(theta1 + theta2) - a1*sin(theta1), -a2*sin(theta1 + theta2);
          a2*cos(theta1 + theta2) + a1*cos(theta1),  a2*cos(theta1 + theta2);];

    %estimated task space position
    mu_e = [a1*cos(theta1) + a2*cos(theta1 + theta2);
            a1*sin(theta1) + a2*sin(theta1 + theta2)];

    %error (task-space error)
    delta = mu_a - mu_e;

    %tolerance check 
    if abs(delta) < 1e-5
        break;
    end

    %revised joint space positions
    q = q + inv(J)*(delta);

    %plotting the values
    O1x = a1*cos(theta1);
    O1y = a1*sin(theta1);

    O2x = mu_e(1);
    O2y = mu_e(2);

    link1 = line([0, O1x], [0, O1y]);
    link2 = line([O1x, O2x], [O1y, O2y]);

    set(link1, 'lineWidth', 2);
    set(link2, 'lineWidth', 5);

    hold on 
     
    plot(O2x, O2y, 'g.', 'markersize', 10);
    plot(0, 0, 'ks', 'markersize', 10)

    axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)]);

    grid on
    pause(0.1);
    delete(link1);
    delete(link2);
end


