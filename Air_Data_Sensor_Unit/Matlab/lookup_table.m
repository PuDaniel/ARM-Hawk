% Compute a lookup table for alpha, beta and q_T in dependence of the three
% measured differential pressures p_12, p_34, p_0S with equal pressure
% spacing.
close all
clear
clc

% Constants from pressure coefficient cp = A'*cos(theta)^2+B'
A = 2.4;
B = -1.4;

% Define the functions and the partial derivatives of them
F1 = @(p_12, alpha, beta, q) p_12 - 2*A*sin(alpha)*cos(alpha)*cos(beta)^2*q;
F2 = @(p_34, alpha, beta, q) p_34 - 2*A*cos(alpha)*sin(beta)*cos(beta)*q;
F3 = @(p_0S, alpha, beta, q) p_0S - (A*cos(alpha)^2*cos(beta)^2 + B)*q;

dF1_alpha = @(alpha, beta, q) 2*A*cos(beta)^2*(sin(alpha)^2 - cos(alpha)^2)*q;
dF1_beta = @(alpha, beta, q) 4*A*sin(alpha)*cos(alpha)*sin(beta)*cos(beta)*q;
dF1_q = @(alpha, beta, q) -2*A*sin(alpha)*cos(alpha)*cos(beta)^2;

dF2_alpha = @(alpha, beta, q) 2*A*sin(alpha)*sin(beta)*cos(beta)*q;
dF2_beta = @(alpha, beta, q) 2*A*cos(alpha)*(sin(beta)^2-cos(beta)^2)*q;
dF2_q = @(alpha, beta, q) -2*A*cos(alpha)*sin(beta)*cos(beta);

dF3_alpha = @(alpha, beta, q) 2*A*sin(alpha)*cos(alpha)*cos(beta)^2*q;
dF3_beta = @(alpha, beta, q) 2*A*cos(alpha)^2*sin(beta)*cos(beta)*q;
dF3_q = @(alpha, beta, q) -A*cos(alpha)^2*cos(beta)^2-B;

% Input pressure values
pressure_step = 50.0;
p_0S_range = 0:pressure_step:2000;
p_12_range = 0:pressure_step:1000;
p_34_range = 0:pressure_step:1000;

% Store iterations and errors to validate solution
iterations = zeros(length(p_12_range), length(p_34_range), length(p_0S_range));
error = zeros(length(p_12_range), length(p_34_range), length(p_0S_range));

% Store alpha, beta & q_T in dependence of p_12, p_34 & p_0S as lookup table
alpha = zeros(length(p_12_range), length(p_34_range), length(p_0S_range));
beta = zeros(length(p_12_range), length(p_34_range), length(p_0S_range));
q_T = zeros(length(p_12_range), length(p_34_range), length(p_0S_range));

for i = 1:length(p_0S_range)
    for j = 1:length(p_12_range)
        for k =1:length(p_34_range)
            p_0S = p_0S_range(i);
            p_12 = p_12_range(j);
            p_34 = p_34_range(k);
            
            % Calulate a first solution with some simplified equations
            q_0 = max([p_0S, abs(p_12), abs(p_34)]);
            
            alpha_0 = 0;
            beta_0 = 0;
            
            if abs(q_0) > 0.1
                alpha_0 = asin(p_12/(2*A*q_0));
                beta_0 = asin(p_34/(2*A*q_0));
            end
            
            x = [alpha_0; beta_0; q_0];
            
            % Iterate until an approximate solution is found
            n = 0;
            
            while 1
                dF = [dF1_alpha(x(1),x(2),x(3)) dF1_beta(x(1),x(2),x(3)) dF1_q(x(1),x(2),x(3));
                    dF2_alpha(x(1),x(2),x(3)) dF2_beta(x(1),x(2),x(3)) dF2_q(x(1),x(2),x(3));
                    dF3_alpha(x(1),x(2),x(3)) dF3_beta(x(1),x(2),x(3)) dF3_q(x(1),x(2),x(3))];
                
                F = -[F1(p_12, x(1),x(2),x(3)); F2(p_34, x(1),x(2),x(3)); F3(p_0S, x(1),x(2),x(3));];
                
                if det(dF) == 0
                    % Trivial solution
                    x = [0; 0; 0];
                    break;
                end
                
                delta = dF\F;
                
                % Do some relaxation
                if abs(delta(3)) > 1000
                    delta = delta*0.5;
                end
                
                x = x + delta;
                
                n = n + 1;
                
                % Check if converged
                if (abs(delta(1)) < 0.05) && (abs(delta(2)) < 0.05) && (abs(delta(3)) < 5)
                    break;
                end
            end
            
            iterations(j, k, i) = n;
            
            % Check the solution
            theta1 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) + 1/sqrt(2)*sin(x(1))*cos(x(2)));
            theta2 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) - 1/sqrt(2)*sin(x(1))*cos(x(2)));
            theta3 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) + 1/sqrt(2)*sin(x(2)));
            theta4 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) - 1/sqrt(2)*sin(x(2)));
            theta0 = acos(cos(x(1))*cos(x(2)));
            
            delta_p12 = (1.2*cos(2*theta1) - 1.2*cos(2*theta2))*x(3);
            delta_p34 = (1.2*cos(2*theta3) - 1.2*cos(2*theta4))*x(3);
            delta_p0S = (1.2*cos(2*theta0) - 0.2)*x(3);
            
            error(j, k, i) = abs(delta_p12 - p_12) + abs(delta_p34 - p_34) + abs(delta_p0S - p_0S);
            
            % Store the aerodynamic parameters
            alpha(j, k, i) = x(1)*180/pi;
            beta(j, k, i) = x(2)*180/pi;
            q_T(j, k, i) = x(3);
        end
        disp([num2str(100*((i - 1)*length(p_12_range) + j)/...
            (length(p_0S_range)*length(p_12_range))) '%'])
    end
end

% Display some ineformation
disp('')
disp('max. Iterations: ')
disp(max(iterations(:)))

disp('max. Error [Pa]: ')
disp(max(error(:)))

disp('Array storage size [kB]: ')
disp(3*4*length(p_0S_range)*length(p_12_range)*length(p_34_range)/1024)

% Write the lookup-table to a text file
fileID = fopen('AirDataComputer_data.cpp', 'w');

fprintf(fileID, '#include "AirDataComputer.hpp"\n\n');
fprintf(fileID, 'namespace ADC_data \n{\n');
fprintf(fileID, '\tfloat pressure_step = %.3f;\n\n', pressure_step);
fprintf(fileID, '\tint p_0S_length = %d;\n', length(p_0S_range));
fprintf(fileID, '\tint p_34_length = %d;\n', length(p_34_range));
fprintf(fileID, '\tint p_12_length = %d;\n\n', length(p_12_range));

% Write alpha
fprintf(fileID, '\tconst float alpha[%d][%d] = \n\t{\n', ...
    [length(p_0S_range), length(p_12_range)*length(p_34_range)]);
for i = 1:length(p_0S_range)
    fprintf(fileID, '\t\t{\n\t\t\t');
    for j = 1:length(p_12_range)
        for k =1:length(p_34_range)
            fprintf(fileID, '%.3f', alpha(j, k, i));
            if (j ~= length(p_12_range)) || (k ~= length(p_34_range))
                fprintf(fileID, ', ');
            end
        end
        if j ~= length(p_12_range)
            fprintf(fileID, '\n\t\t\t');
        else
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '\t\t},\n');
end
fprintf(fileID, '\t};\n\n');

% Write beta
fprintf(fileID, '\tconst float beta[%d][%d] = \n\t{\n', [length(p_0S_range), ...
    length(p_12_range)*length(p_34_range)]);
for i = 1:length(p_0S_range)
    fprintf(fileID, '\t\t{\n\t\t\t');
    for j = 1:length(p_12_range)
        for k =1:length(p_34_range)
            fprintf(fileID, '%.3f', beta(j, k, i));
            if (j ~= length(p_12_range)) || (k ~= length(p_34_range))
                fprintf(fileID, ', ');
            end
        end
        if j ~= length(p_12_range)
            fprintf(fileID, '\n\t\t\t');
        else
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '\t\t},\n');
end
fprintf(fileID, '\t};\n\n');

% Write q_T
fprintf(fileID, '\tconst float q_T[%d][%d] = \n\t{\n', [length(p_0S_range), ...
    length(p_12_range)*length(p_34_range)]);
for i = 1:length(p_0S_range)
    fprintf(fileID, '\t\t{\n\t\t\t');
    for j = 1:length(p_12_range)
        for k =1:length(p_34_range)
            fprintf(fileID, '%.3f', q_T(j, k, i));
            if (j ~= length(p_12_range)) || (k ~= length(p_34_range))
                fprintf(fileID, ', ');
            end
        end
        if j ~= length(p_12_range)
            fprintf(fileID, '\n\t\t\t');
        else
            fprintf(fileID, '\n');
        end
    end
    fprintf(fileID, '\t\t},\n');
end
fprintf(fileID, '\t};\n');
fprintf(fileID, '};\n');

fclose(fileID);

disp('Results saved in "AirDataComputer_data.cpp"')