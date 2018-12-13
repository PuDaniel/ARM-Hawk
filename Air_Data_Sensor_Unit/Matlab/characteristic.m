% Calculate the differential pressures p_12 and p_34 in dependence of alpha
% and beta for some constant dynamic pressures (q_T). Plot the results as
% curves of alpha and beta over p_12 and p_34.
close all
clear
clc

% Calculate the pressure readings
phi = pi/4;

for n = 1:1:1
    u = n*30;
    q = 1.225/2*u^2;

    delta_p12 = zeros(91, 91);
    delta_p34 = zeros(91, 91);

    for alpha = -45:1:45
        for beta = -45:1:45
            theta1 = acos(cos(phi)*cos(alpha*pi/180)*cos(beta*pi/180) + ...
                sin(phi)*sin(alpha*pi/180)*cos(beta*pi/180));
            theta2 = acos(cos(phi)*cos(alpha*pi/180)*cos(beta*pi/180) - ...
                sin(phi)*sin(alpha*pi/180)*cos(beta*pi/180));
            theta3 = acos(cos(phi)*cos(alpha*pi/180)*cos(beta*pi/180) + ...
                sin(phi)*sin(beta*pi/180));
            theta4 = acos(cos(phi)*cos(alpha*pi/180)*cos(beta*pi/180) - ...
                sin(phi)*sin(beta*pi/180));

            delta_p12(alpha+46,beta+46) = (cp(theta1) - cp(theta2))*q;
            delta_p34(alpha+46,beta+46) = (cp(theta3) - cp(theta4))*q;
        end
    end

    % Plot the characteristic of the probe
    %subplot(2,2,n);
    
    for alpha = -45:5:45
        p1 = plot(delta_p12(alpha+46,:), delta_p34(alpha+46,:), 'red');
        text(delta_p12(alpha+46,1), delta_p34(alpha+46,1), num2str(alpha), 'Color', 'red');
        hold on
    end

    for beta = -45:5:45
        p2 = plot(delta_p12(:,beta+46), delta_p34(:,beta+46), 'blue');
        text(delta_p12(1,beta+46), delta_p34(1,beta+46), num2str(beta), 'Color', 'blue');
        hold on
    end

    xlabel('delta p_1_2 [Pa]');
    ylabel('delta p_3_4 [Pa]');
    legend([p1 p2], 'alpha [deg]', 'beta [deg]');
    title([num2str(u), ' [m/s]']);
    grid on
end

function c_p = cp(theta)
    % Calculate pressure coefficient
    c_p = 1.2*cos(2*theta) - 0.2; %1.195*cos(2*(theta - 0.0707)) - 0.185;
end