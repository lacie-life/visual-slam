clear all
close all
rawGPS = table2array(readtable('KittiGps_converted.txt'));
correctedGPS = table2array(readtable('IMUKittiExampleGPSResults.csv'));
correctedGPS = correctedGPS(:,1:4);
correctedGPS = [rawGPS(1,:);correctedGPS];
figure(1)
plot3(rawGPS(:,2),rawGPS(:,3),rawGPS(:,4), 'linewidth', 2);
hold on;
plot3(correctedGPS(:,2),correctedGPS(:,3),correctedGPS(:,4), 'linewidth', 2);
hold off;
axis equal;
grid;
figure(2)
subplot(311)
plot(correctedGPS(:, 1), correctedGPS(:, 2));
hold on;
plot(rawGPS(:, 1), rawGPS(:, 2));
hold off;
grid;
legend('corrected', 'raw');
subplot(312)
plot(correctedGPS(:, 1), correctedGPS(:, 3));
hold on;
plot(rawGPS(:, 1), rawGPS(:, 3));
hold off;
grid;
legend('corrected', 'raw');
subplot(313)
plot(correctedGPS(:, 1), correctedGPS(:, 4));
hold on;
plot(rawGPS(:, 1), rawGPS(:, 4));
hold off;
grid;
legend('corrected', 'raw');
