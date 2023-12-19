%% D.H. Lineardo, CV temp

%Barbie 
%Simulation of a 2-axis planar RR robot

% Display the binary image
%imshow(binaryImage);

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

%importing image 
[orgImage, orgMap] = imread("CIRCLE.png");
orgImage = orgImage(1:1:end, 1:1:end, :);

% Convert indexed image to true-color RGB
trueColorImage = ind2rgb(orgImage, orgMap);

% Convert true-color RGB to grayscale
grayImage = rgb2gray(trueColorImage);

% Convert grayscale image to binary using adaptive thresholding
binaryImage = imbinarize(grayImage, 'adaptive');

% Get the size of the binary image
sizeBi = size(binaryImage);

% White pixels
[rows, cols] = find(binaryImage == 1);

% Combine the rows and columns to get the points
points = [rows, cols];

% Rescale the points to match the arm's length (e.g., 2 units)
armLength = 2;  % Change this value to the desired arm length

% Scale the points to the range [0, 1] and then multiply by arm's length
points = (points - 1) / (max(sizeBi) - 1) * armLength;

% Display or use the scaled points
disp(points);

%inital guess 
q = [pi/3; pi/3];

for i = 1:size(points, 1)  % Loop through each point in points
    % Extract x and y coordinates of the current point
    x = points(i, 1)
    y = points(i, 2)

    % Calculate task space position for the current point
    mu_a = [x; y];

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
    plot(0, 0, 'ks', 'markersize', 10);

    axis([-(a1+a2) (a1+a2) -(a1+a2) (a1+a2)]);

    grid on
    pause(0.1);
    delete(link1);
    delete(link2);
end
