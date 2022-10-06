function [ d12, dInf, P_prime_list ] = dist_ig_mat2( x1,P1,x2_mat,P2_mat,alpha, R, Metric_id )
% This function computes the total cost = Euclidean+ Info gain
% from node (x,P) to nodes in (x2_mat, P2_mat) 
% d12 is the total cost 
% dInf is the information cost
% P_prime_list is the minimizer of the optimization program in the
% definition of total cost 


d_cont = sqrt( sum( (x2_mat - x1 ).^2, 1) );
num_dcont = numel(d_cont);

d12 = zeros(1, num_dcont);
dInf = zeros(1, num_dcont);
P_prime_list = zeros(2, 2, num_dcont);


if Metric_id==1 %Entropy

    for j = 1:num_dcont

        P_hat = P1 + R*d_cont(j);
        P_prime =  P2_mat(:,2*(j-1)+1:2*j); % since transition is lossless


        P_prime( abs(P_prime) < 10^-14 ) = 0;
        P_prime_list(:,:,j) = P_prime;
    
        dInf(1,j) = (alpha/2)*(log2(det(P_hat))-log2(det(P_prime)));
        d12(1,j) = d_cont(j) + dInf(1,j);
    end
end

% margin to avoid 0 
epsilon=1.0e-6;


if Metric_id==2 % Wasserstein distance
    
        for j = 1:num_dcont

            P_hat = P1 + R*d_cont(j);
            P_prime =  P2_mat(:,2*(j-1)+1:2*j); % since transition is lossless


            P_prime( abs(P_prime) < 10^-14 ) = 0;
            P_prime_list(:,:,j) = P_prime;


            P_inter = sqrtm( sqrtm(P_prime) * P_hat *  sqrtm(P_prime));


            dInf(1,j) = (alpha) * sqrt(trace(P_prime + P_hat - 2 * P_inter)+epsilon);

            d12(1,j) = d_cont(j) + dInf(1,j);
       end
end



if Metric_id==3
    
    for j = 1:num_dcont

         P_hat = P1 + R*d_cont(j);
         P_prime =  P2_mat(:,2*(j-1)+1:2*j); % since transition is lossless


         P_prime( abs(P_prime) < 10^-14 ) = 0;
         P_prime_list(:,:,j) = P_prime;
    
        num = (det(P_hat))^(1/4) * (det(P_prime))^(1/4);
        denum = (det(P_hat/2+P_prime/2)) ^ (1/2);

        dInf(1,j) = (alpha) * sqrt(1+epsilon-num/denum);

        d12(1,j) = d_cont(j) + dInf(1,j);
    end
end






end