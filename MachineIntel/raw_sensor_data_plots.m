clear; close all;
dataRaw = importdata('Raw_data_no_integral.log');
xRaw = dataRaw(:, 1:2);
accelZ = xRaw(:, 1);
gyroY = xRaw(:, 2);
rawResponse = dataRaw(:, 3);

dataTrans = importdata('angle_angleRate.log');
xTrans = dataTrans(:, 1:2);
angle = xTrans(:, 1);
angleRate = xTrans(:, 2);
response = dataTrans(:, 4);





%figure(1);
%subplot(1, 3, 1);
%plot(accelZ, response, 'bo');
%xlabel('accel z', FontSize=12)
%ylabel('Motor Response', FontSize=12)
%title("accel z vs. Motor Response");
%grid on;

%subplot(1, 2, 2);
%plot(gyroY, response, 'ro');
%xlabel('accel x', FontSize=12)
%ylabel('Motor Response', FontSize=12)
%title("accel x vs. Motor Response");
%grid on;

figure(1)
scatter3(accelZ, gyroY, rawResponse);
xlabel('Accelerometer Z Axis');
ylabel('Gyroscope Y Axis');
zlabel('Motor Response')
title("Raw Accel and Raw Gyro vs. Motor Response")

figure(2)
scatter3(angle, angleRate, response);
xlabel('Angle Estimation');
ylabel('Angular Rate of Change');
zlabel('Motor Response')
title("Angle and Angle Rate vs. Motor Response")

