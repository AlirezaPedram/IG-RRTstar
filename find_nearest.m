function [ j_min, d_min ] = find_nearest( node, In_list_ID, x, P, R, alpha, Metric_id )
% Find the nearest node from nodes in the tree (In_list_ID) to the 
% node (x,P) and the associated minimum distance

% reshaping
x_node = reshape([node(In_list_ID).x], [2, numel(In_list_ID)]).';
P_node = [node(In_list_ID).P];
%P_rep = repmat(P, [1, numel(In_list_ID)]);

%Euclidean distance
d = sqrt( sum( (x_node - x).^2, 2) );

%Info distance (Frobenius norm)
%dist_P = sqrt( sum( reshape( sum((P_node - P_rep).^2), [2, numel(In_list_ID)] ) ) );

dist_P=zeros(1,length(d));  

if Metric_id==1 % Entropy
 
    for j=1:length(d)

        P_hat = P_node(:,2*(j-1)+1:2*j) + R * d(j);
    
        P_prime = Q_hat_sol(P_hat, P);

        dist_P(1,j) = (alpha/2) * (log2(det(P_hat))-log2(det(P_prime)));
    end
end


% margin to avoid 0 
epsilon=1.0e-6;


if Metric_id==2 % Wasserstein distance
    
    for j=1:length(d)

        P_hat = P_node(:,2*(j-1)+1:2*j) + R * d(j);
    
        % we use this to avoid lossy transition for now
        P_prime = Q_hat_sol(P_hat, P);

        P_inter = sqrtm( sqrtm(P_prime) * P_hat *  sqrtm(P_prime));
    
        dist_P(1,j) = (alpha) * sqrt(trace(P_prime + P_hat- 2* P_inter)+epsilon);
    end
end


if Metric_id==3 % Heliinger distance
    
    for j=1:length(d)

        P_hat = P_node(:,2*(j-1)+1:2*j) + R * d(j);
    
        % we use this to avoid lossy transition for now
        P_prime = Q_hat_sol(P_hat, P);

        num = (det(P_hat))^(1/4) * (det(P_prime)) ^(1/4);
        denum = (det(P_hat/2+P_prime/2)) ^ (1/2);
    
        dist_P(1,j) = (alpha) * sqrt(1+epsilon-num/denum);
    end

end


% Find the minimum 
[d_min, j_min] = min(d + dist_P.');
end
