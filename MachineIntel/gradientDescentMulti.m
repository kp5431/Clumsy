function [theta, J_history] = gradientDescentMulti(Xdata, y, theta, alpha, num_iters)
%GRADIENTDESCENT Performs gradient descent to learn theta
%   theta = GRADIENTDESENT(Xdata, y, theta, alpha, num_iters) updates theta by 
%   taking num_iters gradient steps with learning rate alpha
% Input:
%   Xdata- input data, size nxD
%   Y- target Y values for input data
%   theta- initial theta values, size Dx1
%   alpha- learning rate
%   num_iters- number of iterations 
%       Where n is the number of samples, and D is the dimension 
%       of the sample plus 1 (the plus 1 accounts for the constant column)
% Output:
%   theta- the learned theta
%   J_history- The least squares cost after each iteration

% Initialize some useful values
    n = length(y); % number of training examples
    J_history = zeros(num_iters, 1);
    
    for iter = 1:num_iters
        thetaTemp = theta;
        for i = 1:length(theta)
            thetaTemp(i) = theta(i) - (alpha/n)*sum((Xdata*theta-y).*Xdata(:,i));
        end
        theta = thetaTemp;
        %save cost history
        J_history(iter) = computeCost(Xdata, y, theta);
    end


end
