clear; close all;

data = importdata('approxAngle.log');
x = data(:, 1:2);
y = data(:, 3);

% Randomize indices of data
idx = randperm(length(x));
x = x(idx, :); % reorder rows
y = y(idx);

%[x mu sigma] = featureNormalize(x);


% Add intercept term to x
xData = [ones(length(x), 1) x];

% Perform k-fold cross-validation
k = 2; % number of folds
cv = cvpartition(size(xData,1), 'KFold', k);
theta_lm_all = zeros(size(xData, 2), k);
theta_svr_all = zeros(size(xData, 2), k);

for i = 1:k
    % Get indices for training and testing sets
    idxTrain = cv.training(i);
    idxTest = cv.test(i);
    
    % Split data into training and testing sets
    xTrain = xData(idxTrain, :);
    yTrain = y(idxTrain);
    xTest = xData(idxTest, :);
    yTest = y(idxTest);

    % Perform linear regression
    lm = fitlm(xTrain, yTrain, 'Intercept', false);
    theta_lm_all(:, i) = lm.Coefficients.Estimate;

    % Perform support vector regression
    %mdl = fitrsvm(xTrain, yTrain, 'KernelFunction', 'rbf', 'OptimizeHyperparameters', 'auto');
    %theta_svr_all(:, i) = mdl.SupportVectors' * mdl.Alpha;
    
    % Train regression tree
    mdl = fitrtree(xTrain, yTrain);

    % Predict on test set
    yPredRTree = predict(mdl, xTest);


    yPredLM = predict(lm, xTest);
    %yPredSVR = predict(mdl, xTest);

    mseLM = mean(((yTest - yPredLM).^2));
    %mseSVR = mean(((yTest - yPredSVR).^2));
    mseRTree = mean(((yTest - yPredRTree).^2));
    fprintf('mseLM: %.4f\n', mseLM);
    %fprintf('mseSVR: %.4f\n', mseSVR);
    fprintf('mseRTree: %.4f\n', mseRTree);
end

% Calculate mean of the theta values from k-fold cross-validation
theta_lm_mean = mean(theta_lm_all, 2);
%theta_svr_mean = mean(theta_svr_all, 2);


% Print results
disp('Linear regression coefficients:')
disp(theta_lm_mean);
%disp('Support vector regression coefficients:')
%disp(theta_svr_mean);
