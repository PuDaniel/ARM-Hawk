function [x_kplus1] = kalman_trans_state(x_k, w_k, u_k)

Ts = 1/400;

fx = u_k(1)+w_k(1);
fy = u_k(2)+w_k(2);
fz = u_k(3)+w_k(3);

P = u_k(4);
Q = u_k(5);
R = u_k(6);

q0 = u_k(7);
q1 = u_k(8);
q2 = u_k(9);
q3 = u_k(10);

g = 9.8082;

F = Ts*[1/Ts                    R                       -Q                      0       0       0;
        -R                      1/Ts                    P                       0       0       0;
        Q                       -P                      1/Ts                    0       0       0;
        (q0^2+q1^2-q2^2-q3^2)   2*(q1*q2-q0*q3)         2*(q1*q3+q0*q2)         1/Ts    0       0;
        2*(q1*q2+q0*q3)         (q0^2-q1^2+q2^2-q3^2)   2*(q2*q3-q0*q1)         0       1/Ts    0;
        2*(q1*q3-q0*q2)         2*(q2*q3+q0*q1)         (q0^2-q1^2-q2^2+q3^2)   0       0       1/Ts];

u = Ts*[fx+2*g*(q1*q3-q0*q2); fy+2*g*(q2*q3+q0*q1); fz+g*(q0^2-q1^2-q2^2+q3^2); 0; 0; 0];
      
x_kplus1 = F*x_k + u;
end