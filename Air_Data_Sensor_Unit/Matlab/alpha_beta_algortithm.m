% An iterative algorithm to compute alpha, beta and q_T from the three
% measured differential pressures, p_12, p_34 and p_0S
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
p_0S = -150;
p_12 = 900;
p_34 = -800;

% Calulate a first solution with some simplified equations
q_0 = max([p_0S, abs(p_12), abs(p_34)]);

alpha_0 = 0;
beta_0 = 0;

if abs(q_0) > 0.1
    alpha_0 = asin(p_12/(2*A*q_0));
    beta_0 = asin(p_34/(2*A*q_0));
end

x = [alpha_0; beta_0; q_0];

x_vect = x;

% Iterate until approximate solution found (Newton method)
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
    
    x_vect = [x_vect x];
    
    % Check if converged
    if (abs(delta(1)) < 0.05) && (abs(delta(2)) < 0.05) && (abs(delta(3)) < 5)
        break;
    end
end

% Plot the trajectory of the solution
plot(0:n, x_vect(1,:), 0:n, x_vect(2,:), 0:n, x_vect(3,:)/1000)

% Print aerodynamic parameters
disp('n=')
disp(n)
disp('alpha [degree]=')
disp(x(1)*180/pi)
disp('beta [degree]=')
disp(x(2)*180/pi)
disp('q [Pa]=')
disp(x(3))

% Check the solution
theta1 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) + 1/sqrt(2)*sin(x(1))*cos(x(2)));
theta2 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) - 1/sqrt(2)*sin(x(1))*cos(x(2)));
theta3 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) + 1/sqrt(2)*sin(x(2)));
theta4 = acos(1/sqrt(2)*cos(x(1))*cos(x(2)) - 1/sqrt(2)*sin(x(2)));
theta0 = acos(cos(x(1))*cos(x(2)));

delta_p12 = (1.2*cos(2*theta1) - 1.2*cos(2*theta2))*x(3);
delta_p34 = (1.2*cos(2*theta3) - 1.2*cos(2*theta4))*x(3);
delta_p0S = (1.2*cos(2*theta0) - 0.2)*x(3);

error = abs(delta_p12 - p_12) + abs(delta_p34 - p_34) + abs(delta_p0S - p_0S);

disp('p_12 [Pa]=')
disp(delta_p12)
disp('p_34 [Pa]=')
disp(delta_p34)
disp('p_0S [Pa]=')
disp(delta_p0S)
disp('Absolute error [Pa]=')
disp(error)