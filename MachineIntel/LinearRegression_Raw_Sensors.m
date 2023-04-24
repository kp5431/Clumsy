clear; close all;

data = importdata('on_rug_accel_gyro_no_disturb.log');
x = data(:, 1:2); %accelZ, gyroY, integral
y = data(:, 3);

%randomise indices of data
idx = randperm(length(x));
x = x(idx, :); %reorder rows
y = y(idx);

%normalize data
[xNorm mu sigma] = featureNormalize(x);

%Add intercept term to x
xData = [ones(length(x), 1) xNorm];

alpha = 1; %learning rate
iters = 40000; %grad descent iterations;

theta = zeros(3, 1);
[theta, J_history] = gradientDescentMulti(xData, y, theta, alpha, iters);

fprintf('Gradient Descent theta = [intercept: %f, aZ: %f, gY: %f\n',theta);
figure;
plot(J_history, '-b', 'LineWidth', 3);
xlabel('Number of iterations', 'fontsize',12);
ylabel('Cost J', 'fontsize',12);
title('Cost over 400 iterations', 'fontsize',14);
grid on