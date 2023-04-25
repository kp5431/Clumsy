data = importdata('angleTest.log');
dataPD = importdata('PDTest.log');
dataApprox = importdata('angleApproxTest.log')

figure(1)
subplot(1,3,1)
plot(data(1:2780));
title("Hand Tuned PID")
ylim([-6 12])

subplot(1,3,2)
plot(dataPD(1:2780));
title("Hand Tuned PD")
ylim([-6 12])

subplot(1,3,3)
plot(dataApprox(1:2780));
title("Less Preprocessing Approximated PD")
ylim([-6 12])