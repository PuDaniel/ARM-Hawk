function [y_k] = kalman_trans_output(x_k, v_k, u_k)

y_k = [x_k(4)+v_k(1); x_k(5)+v_k(2); x_k(6)+v_k(3)];
end