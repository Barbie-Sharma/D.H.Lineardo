%Barbie 
%Simulation of a 2-axis planar RR robot

clc 
clearvars
clf

%symbolic variables
syms theta1 theta2

l1 = 1;
l2 = 1;

%DH parameters 

%link 1
a1 = l1;
alpha1 = 0;
d1 = 0;
theta1 = theta1;

%link 2
a2 = l2;
alpha2 = 0;
d2 = 0;
theta2 = theta2;

H0_1 = DH(a1, alpha1, d1, theta1);
H1_2 = DH(a2, alpha2, d2, theta2);

O1x = H0_1(1, 4);
O1y = H0_1(2, 4);

O2x = simplify(H0_1(1, 4));
O2y = simplify(H0_1(2, 4));

t = 0;
dt = 0.01;

xlim([-2, 2]);
ylim([-2, 2]);

grid on

for i = 0:360
    theta1 = i;
    theta2 = i;

    O1x = cosd(theta1);
    O1y = sind(theta1);

    O2x = cosd(theta1 + theta2)/2 + cosd(theta1);
    O2y = sind(theta1 + theta2)/2 + sind(theta2);

    link1 = line([0, O1x], [0, O1y]);
    link2 = line([O1x, O2x], [O1y, O2y]);

    set(link1, 'lineWidth', 2);
    set(link2, 'lineWidth', 5);

    hold on 
     
    plot(O2x, O2y, 'g.');

    pause(0.1);

    delete(link1);
    delete(link2);
end 
