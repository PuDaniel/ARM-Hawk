function [C, omega_0, G] = gyro_calibration(theta_gyro, T)
% Calibrate the gyrometer:
% omega_v = C*(omega_gyro - omega_0) - G*f
% @param    theta_gyro  3x24 matrix consisting of measured and integrated angle vectors [rad/s]
% @param    T           1x24 vector consisting of integration times [s]
% @return   C           Calibration matrix (3x3)
% @return   omega_0     Angular rate bias vector [rad/s]
% @return   G           G-force coupling matrix [rad*s/m]

%% Define the test angle vectors and the gravitational acceleration vectors
theta = pi;

theta_x = -theta/sqrt(2) * [1 -1  0  0 -1  1  0  0  1 -1 -1  1 -1  1  1 -1  1 -1  0  0 -1  1  0  0]';
theta_y = -theta/sqrt(2) * [0  0  1 -1  0  0 -1  1  1 -1  1 -1 -1  1 -1  1  0  0  1 -1  0  0 -1  1]';
theta_z = -theta/sqrt(2) * [1 -1  1 -1  1 -1  1 -1  0  0  0  0  0  0  0  0 -1  1 -1  1 -1  1 -1  1]';

g = 9.808237;

f_x = -g/sqrt(2) * [1  1  0  0 -1 -1  0  0  1  1 -1 -1 -1 -1  1  1  1  1  0  0 -1 -1  0  0]';
f_y = -g/sqrt(2) * [0  0  1  1  0  0 -1 -1  1  1  1  1 -1 -1 -1 -1  0  0  1  1  0  0 -1 -1]';
f_z = -g/sqrt(2) * [1  1  1  1  1  1  1  1  0  0  0  0  0  0  0  0 -1 -1 -1 -1 -1 -1 -1 -1]';

%% Rearrange the input data
Omega_x = [theta_gyro' -T' -T'.*f_x];
Omega_y = [theta_gyro' -T' -T'.*f_y];
Omega_z = [theta_gyro' -T' -T'.*f_z];

%% Compute the calibration matrix and the bias vector
c_1 = (Omega_x'*Omega_x)\(Omega_x'*theta_x);
c_2 = (Omega_y'*Omega_y)\(Omega_y'*theta_y);
c_3 = (Omega_z'*Omega_z)\(Omega_z'*theta_z);

C = [c_1(1) c_1(2) c_1(3);
     c_2(1) c_2(2) c_2(3);
     c_3(1) c_3(2) c_3(3)];

omega_0 = C\[c_1(4); c_2(4); c_3(4)];

G = [c_1(5)   0      0;
       0    c_2(5)   0;
       0        0  c_3(5)];

%% Print the calibration results
format shortE

if exist('gyro_calibration_log.txt', 'file')
    delete('gyro_calibration_log.txt');
end
diary('gyro_calibration_log.txt')

disp(' ')
disp('Calibration form: omega_v = C*(omega_gyro - omega_0) - G*f')
disp(' ')
disp('Calibration matrix C=')
disp(C)
disp('Angular rate bias vector omega_0 [rad/s]=')
disp(omega_0)
disp('G-force coupling matrix G [rad*s/m]=')
disp(G)

%% Print the calibration error
error = norm(mean(abs([theta_x'; theta_y'; theta_z'] - theta_gyro), 2))/theta*100;
disp(' ')
disp('Error before calibration [%]=')
disp(error)

error = norm(mean(abs([theta_x'; theta_y'; theta_z'] - (C*(theta_gyro - omega_0*T) - G*[T.*f_x'; T.*f_y'; T.*f_z'])), 2))/theta*100;
disp('Error after calibration [%]=')
disp(error)

diary off

format
end

