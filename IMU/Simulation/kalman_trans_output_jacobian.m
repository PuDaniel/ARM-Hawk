function [H_k,M_k] = kalman_trans_output_jacobian(x_k, v_k, u_k)

H_k = [0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];

M_k = [1 0 0;
       0 1 0;
       0 0 1];
end

