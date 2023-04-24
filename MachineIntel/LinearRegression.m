clear; close all;

data = importdata('approxAngle.log');
x = data(:, 1:2); %angle, angleRate, integral
y = data(:, 3);

%randomise indices of data
idx = randperm(length(x));
x = x(idx, :); %reorder rows
y = y(idx);

%Add intercept term to x
xData = [ones(length(x), 1) x];

alpha = 0.00001; %learning rate
iters = 10000; %grad descent iterations;

theta = zeros(3, 1);
[theta, J_history] = gradientDescentMulti(xData, y, theta, alpha, iters);

fprintf('Gradient Descent theta = [intercept: %f, a: %f, aR: %f, int: %f]\n',theta);
figure;
plot(J_history, '-b', 'LineWidth', 3);
xlabel('Number of iterations', 'fontsize',12);
ylabel('Cost J', 'fontsize',12);
title('Cost over 10000 iterations', 'fontsize',14);
grid on