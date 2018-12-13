function [B_I] = mag_calibration_current(C, B_0, B_M_north, B_M_east, B_M_south, B_M_west, I, B_M_north_I, B_M_east_I, B_M_south_I, B_M_west_I)
% Compute the motor current dependet bias of the magnetometer:
% B_M = S*M*B_M + B_0 + I*B_I
% @param    C           Precomputed calibration matrix (3x3)
% @param    B_0         Precomputed hard-iron offset vector [T]
% @param    B_M_north   Measured static geographical north flux vector [T]
% @param    B_M_east    Measured static geographical east flux vector [T]
% @param    B_M_south   Measured static geographical south flux vector [T]
% @param    B_M_west    Measured static geographical west flux vector [T]
% @return   I           Motor current at measurement run [A]
% @param    B_M_north_I During motor run measured geographical north flux vector [T]
% @param    B_M_east_I  During motor run measured geographical east flux vector [T]
% @param    B_M_south_I During motor run measured geographical south flux vector [T]
% @param    B_M_west_I  During motor run measured geographical west flux vector [T]
% @return   B_I         Current dependent hard-iron offset vector [T/A]

%% Read in geomagnetic data from a NGDC .csv file
earth_mag = csvread('igrfwmmData.csv', 21, 0);
B_E = [earth_mag(6); earth_mag(7); earth_mag(8)] * 1e-9;

%% Compute the current dependent bias by averaging the difference in magnetic flux density
B_I = mean([(B_M_north_I - B_M_north)/I (B_M_east_I - B_M_east)/I ...
        (B_M_south_I - B_M_south)/I (B_M_west_I - B_M_west)/I], 2);

%% Print the calibration results
format shortE

if exist('mag_calibration_current_log.txt', 'file')
    delete('mag_calibration_current_log.txt');
end
diary('mag_calibration_current_log.txt')

disp(' ')
disp('Calibration form: B_M = S * M * B_E + B_0 + I*B_I')
disp('                  B_E = C * (B_M - B_0 - I*B_I)')
disp(' ')
disp('Current dependent hard-iron offset vector B_I [T/A]=')
disp(B_I)

%% Print the calibration error
psi_E = -[atan2(B_E(2), B_E(1));
          atan2(-B_E(1), B_E(2));
          atan2(-B_E(2), -B_E(1));
          atan2(B_E(1), -B_E(2))];
B_Mc_north = C*(B_M_north_I-B_0);
B_Mc_east = C*(B_M_east_I-B_0);
B_Mc_south = C*(B_M_south_I-B_0);
B_Mc_west = C*(B_M_west_I-B_0);
psi_M = -[atan2(B_Mc_north(2), B_Mc_north(1));
          atan2(B_Mc_east(2), B_Mc_east(1));
          atan2(B_Mc_south(2), B_Mc_south(1));
          atan2(B_Mc_west(2), B_Mc_west(1))];
error = mean(abs(psi_E - psi_M))/(2*pi)*100;
disp(' ')
disp('Error before calibration [%]=')
disp(error)

B_Mc_north = C*(B_M_north_I-B_0-I*B_I);
B_Mc_east = C*(B_M_east_I-B_0-I*B_I);
B_Mc_south = C*(B_M_south_I-B_0-I*B_I);
B_Mc_west = C*(B_M_west_I-B_0-I*B_I);
psi_M = -[atan2(B_Mc_north(2), B_Mc_north(1));
          atan2(B_Mc_east(2), B_Mc_east(1));
          atan2(B_Mc_south(2), B_Mc_south(1));
          atan2(B_Mc_west(2), B_Mc_west(1))];
error = mean(abs(psi_E - psi_M))/(2*pi)*100;
disp('Error before calibration [%]=')
disp(error)

diary off

format
end

