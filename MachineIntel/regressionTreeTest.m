clear; close all;

% Load example data
load carsmall

% Define predictor and response variables
X = [Weight, Horsepower];
Y = MPG;

% Split data into training and testing sets
cv = cvpartition(length(Y), 'HoldOut', 0.2);
Xtrain = X(cv.training,:);
Ytrain = Y(cv.training,:);
Xtest = X(cv.test,:);
Ytest = Y(cv.test,:);

% Train regression tree
mdl = fitrtree(Xtrain, Ytrain);

% Predict on test set
Ypred = predict(mdl, Xtest);

% Calculate mean squared error
MSE = mean((Ypred - Ytest).^2);

% Plot regression tree
view(mdl, 'Mode', 'graph');