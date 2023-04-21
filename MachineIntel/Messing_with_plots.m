data = importdata('on_rug_new_tune.log');
angle = data(:, 1);
angleRate = data(:, 2);
integral = data(:, 3);
response = data(:, 4);

figure(1);
subplot(1, 3, 1);
plot(angle, response, 'bo');
xlabel('Angle Estimate (Degrees)', FontSize=12)
ylabel('Motor Response', FontSize=12)
title("Angle Estimate vs. Motor Response");
grid on;

subplot(1, 3, 2);
plot(angleRate, response, 'ro');
xlabel('AngleRate (Degrees Per Second)', FontSize=12)
ylabel('Motor Response', FontSize=12)
title("AngleRate vs. Motor Response");
grid on;

subplot(1,3,3);
plot(integral, response, 'yo');
xlabel('Integral', FontSize=12)
ylabel('Motor Response', FontSize=12)
title("Integrated Angle vs. Motor Response");
grid on;


