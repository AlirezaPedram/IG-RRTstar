function not_lossless = check_lossless(x_st, P_st, x_end, P_end, R)
% This function checks if the transtion from node (x_st, P_st)
% to the node (x_end,P_end) is lossless or not

eps = 0.000001;
P_hat = P_st + R * norm( x_end - x_st );
not_lossless_mat = -(P_hat - P_end);
lam_Notless = eig(not_lossless_mat);
not_lossless = any(lam_Notless > eps);