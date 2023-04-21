function J = computeCost(xData, y, theta)
    J = sum((xData*theta -y).^2) /(2*length(y));
end
