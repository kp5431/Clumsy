data = importdata('Capture1NoDisturbances.log');
x = data(:, 1);
y = data(:, 2);
plot(x, y, 'bo');
xlabel('Angle Estimate (Degrees)', FontSize=12)
ylabel('Motor Response', FontSize=12)
title("Angle Estimate vs. Motor Response");
grid on;