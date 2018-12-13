function [C, f_0] = acc_calibration(f_acc)
% Calibrate the accelerometer:
% f = C*(f_acc - f_0)
% @param    f_acc       3x18 matrix consisting of measured acceleration vectors [m/s^2]
% @return   C           Calibration matrix (3x3)
% @return   B_0         Acceleration bias vector [m/s^2]

%% Define the local gravitational acceleration and the test vectors
g = 9.808237;

f_x = -[g/sqrt(2) * [1  0 -1  0  1 -1 -1  1  1  0 -1  0]  g * [1 -1  0  0  0  0]]';
f_y = -[g/sqrt(2) * [0  1  0 -1  1  1 -1 -1  0  1  0 -1]  g * [0  0  1 -1  0  0]]';
f_z = -[g/sqrt(2) * [1  1  1  1  0  0  0  0 -1 -1 -1 -1]  g * [0  0  0  0  1 -1]]';

%% Rearrange the input data
F_acc = [f_acc' -ones(18, 1)];

%% Compute the calibration matrix and the bias vector
c_1 = (F_acc'*F_acc)\(F_acc'*f_x);
c_2 = (F_acc'*F_acc)\(F_acc'*f_y);
c_3 = (F_acc'*F_acc)\(F_acc'*f_z);

C = [c_1(1) c_1(2) c_1(3);
     c_2(1) c_2(2) c_2(3);
     c_3(1) c_3(2) c_3(3)];

f_0 = C\[c_1(4); c_2(4); c_3(4)];

%% Print the calibration results
format shortE

if exist('acc_calibration_log.txt', 'file')
    delete('acc_calibration_log.txt');
end
diary('acc_calibration_log.txt')

disp(' ')
disp('Calibration form: f = C*(f_acc - f_0)')
disp(' ')
disp('Local magnitude of gravity vector [m/s^2]=')
disp(g)
disp(' ')
disp('Calibration matrix C=')
disp(C)
disp('Acceleration bias vector f_0 [m/s^2]=')
disp(f_0)

%% Print the calibration error
error = norm(mean(abs([f_x'; f_y'; f_z'] - f_acc), 2))/g*100;
disp(' ')
disp('Error before calibration [%]=')
disp(error)

error = norm(mean(abs([f_x'; f_y'; f_z'] - C*(f_acc - f_0)), 2))/g*100;
disp('Error after calibration [%]=')
disp(error)

diary off

format
end

