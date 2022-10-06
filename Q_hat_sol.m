function Q_hat_sol = Q_hat_sol(P_hat, P_2)
%This funnction computes an ellispe with maximum volume
% thayt is smaller than both P_hat and P_2

P_intermed = inv(sqrtm(P_2)) * P_hat * inv(sqrtm(P_2));

[U,D] =  eig(P_intermed);



S = min(D, eye(length(D)));

Q_hat_sol = sqrtm(P_2) * U * S * U' * sqrtm(P_2);
end