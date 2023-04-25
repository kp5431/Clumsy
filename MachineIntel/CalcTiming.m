dataApprox = importdata('approxAngleTime.log');
dataPD = importdata('PDTime.log');
fprintf('mean time for approx: %.4fus\n', mean(dataApprox));
fprintf('mean time for PD: %.4fus\n', mean(dataPD));
fprintf('Time saved: %.4us\n', abs(mean(dataPD) - mean(dataApprox)));