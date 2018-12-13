function [C, B_0] = mag_calibration(B_Mx, B_My, B_Mz, B_M_north, B_M_east, B_M_south, B_M_west)
% Calibrate the magnetometer:
% B_M = S*M*B_M + B_0
% @param    B_Mx        x values of measured magnetic flux density [T]
% @param    B_My        y values of measured magnetic flux density [T]
% @param    B_Mz        z values of measured magnetic flux density [T]
% @param    B_M_north   Measured geographical north flux vector [T]
% @param    B_M_east    Measured geographical east flux vector [T]
% @param    B_M_south   Measured geographical south flux vector [T]
% @param    B_M_west    Measured geographical west flux vector [T]
% @return   C           Calibration matrix (3x3)
% @return   B_0         Hard-iron offset vector [T]

%% Read in geomagnetic data from a NGDC .csv file
earth_mag = csvread('igrfwmmData.csv', 21, 0);
B_E = [earth_mag(6); earth_mag(7); earth_mag(8)] * 1e-9;

%% Process the input data
n = length(B_Mx);

B_Mx = reshape(B_Mx, n, 1);
B_My = reshape(B_My, n, 1);
B_Mz = reshape(B_Mz, n, 1);

%% Compute the hard-iron offset vector by optimizing the ellipsoid interpolation
error = @(B_0) ellipsoid_error(B_Mx - B_0(1), B_My - B_0(2), B_Mz - B_0(3));

B_0 = [(max(B_Mx) + min(B_Mx))/2;
       (max(B_My) + min(B_My))/2;
       (max(B_Mz) + min(B_Mz))/2];

B_0 = fminsearch(error, B_0);

%% Compute the scale matrix by deforming the ellipsoid back into a sphere
B_Mx_ = B_Mx - B_0(1);
B_My_ = B_My - B_0(2);
B_Mz_ = B_Mz - B_0(3);

b = ones(n, 1);
B = [B_Mx_.^2 B_My_.^2 B_Mz_.^2 2*B_Mx_.*B_My_ 2*B_Mx_.*B_Mz_ 2*B_My_.*B_Mz_];

a = (B'*B)\(B'*b);

A = [a(1) a(4) a(5);
    a(4) a(2) a(6);
    a(5) a(6) a(3)];

S = 1/norm(B_E) * A^(-1/2);

%% Compute the misalignment matrix by performing a quaternion rotation
a = [B_E(1)  B_E(2) -B_E(1) -B_E(2);
     B_E(2) -B_E(1) -B_E(2)  B_E(1);
     B_E(3)  B_E(3)  B_E(3)  B_E(3)];

b = S\([B_M_north B_M_east B_M_south B_M_west] - B_0);

P = a*b';

M = (P*(P'*P)^(-1/2))';

%% Compute the calibration matrix
C = (S*M)^(-1);

%% Print the calibration results
format shortE

if exist('mag_calibration_log.txt', 'file')
    delete('mag_calibration_log.txt');
end
diary('mag_calibration_log.txt')

disp(' ')
disp('Calibration form: B_M = S * M * B_E + B_0')
disp('                  B_E = (S * M)^(-1) * (B_M - B_0)')
disp('                  B_E = C * (B_M - B_0)')
disp(' ')
disp('Local magnetic field vector [T]=')
disp(B_E)
disp(' ')
disp('Scale matrix S=')
disp(S)
disp('Misalignment matrix M=')
disp(M)
disp('Calibration matrix C=')
disp(C)
disp('Hard-iron offset vector B_0 [T]=')
disp(B_0)

%% Print the calibration error
psi_E = -[atan2(B_E(2), B_E(1));
          atan2(-B_E(1), B_E(2));
          atan2(-B_E(2), -B_E(1));
          atan2(B_E(1), -B_E(2))];
psi_M = -[atan2(B_M_north(2)-B_0(2), B_M_north(1)-B_0(1));
          atan2(B_M_east(2)-B_0(2), B_M_east(1)-B_0(1));
          atan2(B_M_south(2)-B_0(2), B_M_south(1)-B_0(1));
          atan2(B_M_west(2)-B_0(2), B_M_west(1)-B_0(1))];
error = mean(abs(psi_E - psi_M))/(2*pi)*100;
disp(' ')
disp('Error before calibration [%]=')
disp(error)

B_Mc_north = C*(B_M_north-B_0);
B_Mc_east = C*(B_M_east-B_0);
B_Mc_south = C*(B_M_south-B_0);
B_Mc_west = C*(B_M_west-B_0);
psi_M = -[atan2(B_Mc_north(2), B_Mc_north(1));
          atan2(B_Mc_east(2), B_Mc_east(1));
          atan2(B_Mc_south(2), B_Mc_south(1));
          atan2(B_Mc_west(2), B_Mc_west(1))];
error = mean(abs(psi_E - psi_M))/(2*pi)*100;
disp('Error before calibration [%]=')
disp(error)

diary off

format

%% Plot the calibration results
close all

% Input data as point cloud
plot3(B_Mx, B_My, B_Mz, '.b')
hold on

% Interpolated ellipsoid
[x, y, z] = sphere(100);

for m = 1:size(x, 1)
    for n = 1:size(x, 2)
        b = S*sqrt(B_E'*B_E)*[x(m, n); y(m, n); z(m, n)] + B_0;
        x(m, n) = b(1);
        y(m, n) = b(2);
        z(m, n) = b(3);
    end
end

surf(x, y, z, 'EdgeColor', 'black', 'EdgeAlpha', .3, 'FaceColor', 'cyan', 'FaceAlpha', .2)

% Ellipsoid center
plot3(B_0(1), B_0(2), B_0(3), '+k', 'MarkerSize', 15)

% North, east, south and west measurement vector
plot3([B_0(1) B_M_north(1)], [B_0(2) B_M_north(2)], [B_0(3) B_M_north(3)], ':k', 'LineWidth', 2)
plot3([B_0(1) B_M_east(1)], [B_0(2) B_M_east(2)], [B_0(3) B_M_east(3)], 'k', 'LineWidth', 2)
plot3([B_0(1) B_M_south(1)], [B_0(2) B_M_south(2)], [B_0(3) B_M_south(3)], 'k', 'LineWidth', 2)
plot3([B_0(1) B_M_west(1)], [B_0(2) B_M_west(2)], [B_0(3) B_M_west(3)], 'k', 'LineWidth', 2)

% Calibrated input data
B_cal = C*[B_Mx' - B_0(1); B_My' - B_0(2); B_Mz' - B_0(3)];
plot3(B_cal(1, :), B_cal(2, :), B_cal(3, :), '.r')

% Earth magnetic field sphere
[x, y, z] = sphere(100);

x = sqrt(B_E'*B_E)*x;
y = sqrt(B_E'*B_E)*y;
z = sqrt(B_E'*B_E)*z;

surf(x, y, z, 'EdgeColor', 'black', 'EdgeAlpha', .3, 'FaceColor', 'magenta', 'FaceAlpha', .2)

% Sphere center
plot3(0, 0, 0, '+k', 'MarkerSize', 15)

% North vector
plot3([0 B_E(1)], [0 B_E(2)], [0 B_E(3)], ':k', 'LineWidth', 2)
plot3([0 B_E(2)], [0 -B_E(1)], [0 B_E(3)], 'k', 'LineWidth', 2)
plot3([0 -B_E(1)], [0 -B_E(2)], [0 B_E(3)], 'k', 'LineWidth', 2)
plot3([0 -B_E(2)], [0 B_E(1)], [0 B_E(3)], 'k', 'LineWidth', 2)

grid on
box on
daspect([1 1 1])
xlabel('B_x [T]')
ylabel('B_y [T]')
zlabel('B_z [T]')

savefig('mag_calibration_plot')
end

%% Helper function for calculating the hard iron offset by computing the ellipsoid fit error
function error = ellipsoid_error(B_Mx_, B_My_, B_Mz_)
b = ones(length(B_Mx_), 1);
B = [B_Mx_.^2 B_My_.^2 B_Mz_.^2 2*B_Mx_.*B_My_ 2*B_Mx_.*B_Mz_ 2*B_My_.*B_Mz_];

a = (B'*B)\(B'*b);

error = sum((B*a - 1).^2);
end