%Barbie 
%% D.H. Lineardo, inversed 

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

%target
theta1 = 0;
theta2 = pi/2;

%foreward kinematics
mu_a = [a1*cos(theta1) + a2*cos(theta1 + theta2);
        a1*sin(theta1) + a2*sin(theta1 + theta2);];

%inital guess 
q = [pi/3; pi/3];

%newton-rapson method
for i = 1:10
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
    plot(mu_a(1), mu_a(2), 'r*', 'markersize', 10)
    plot(0, 0, 'ks', 'markersize', 10)

    axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)]);

    grid on
    pause(3);
    delete(link1);
    delete(link2);
end


