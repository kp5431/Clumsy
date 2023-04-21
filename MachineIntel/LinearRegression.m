clear; close all;

data = importdata('on_rug_new_tune.log');
x = data(:, 1:3); %angle, angleRate, integral
y = data(:, 4);

%Add intercept term to x
xData = [ones(length(x), 1) x];

alpha = 0.0001; %learning rate
iters = 10000; %grad descent iterations;

theta = zeros(4, 1);
[theta, J_history] = gradientDescentMulti(xData, y, theta, alpha, iters);

fprintf('Gradient Descent theta = [intercept: %f, a: %f, aR: %f, int: %f]\n',theta);
figure;
plot(J_history, '-b', 'LineWidth', 3);
xlabel('Number of iterations', 'fontsize',12);
ylabel('Cost J', 'fontsize',12);
title('Cost over 10000 iterations', 'fontsize',14);
grid on