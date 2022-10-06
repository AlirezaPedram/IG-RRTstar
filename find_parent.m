function [parent, value, nbor_issue, P_prime_parent2node_i] =...
    find_parent( x, P, node, R, alpha, radius, obs_x, chi, bound, num_props, prop, In_list_ID, nearest, Metric_id)
% find the parent for the sampled node (x,P)
% parent: ID of the parent node
% nbor_issue = true means no parent is found
% P_prime_parent2node_i: the minimizer of the optimization
% program defined for total cost
% P_prime_parent2node_i is also the loss-less modification of P 

% Definition and initial values for outputs
nbor_issue = 0;
parent = 0;
value = 0;
P_prime_parent2node_i = -ones(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check if the transition from the nearest node is collision-free
issue_flag = psuedo_obs_check_line_oct(x, P, node(nearest), obs_x, R, chi, bound, num_props, prop);

% computes the value and the p_hat if nearest is the parent of samples
% point (x,P)
if issue_flag==0    
dim = numel(x);
x_nearest_vec = [node(nearest).x].';
x_nearest_mat = reshape( x_nearest_vec, [dim, numel(x)/dim]);
P_nearest_mat = [node(nearest).P];
[dig, ~, P_prime_list] = dist_ig_mat( x_nearest_mat, P_nearest_mat, x.', P, alpha, R, Metric_id);
parent = nearest;
value = [node(nearest).value].' + dig.';
P_prime_parent2node_i = P_prime_list;
end


% check if nearest is the only neighbor
Is_only_neibor_ID = false;
[nbors_ID, dist_xk2node_i,~ , P_prime_nei_list] =...
    find_neighbors( x, P, node, R, alpha, radius, In_list_ID, Is_only_neibor_ID, true, Metric_id); 


if isempty(nbors_ID) == 1 && issue_flag==1
   nbor_issue = 1; % There is no neighbors
else
%%%%%%%%%%%%%%%%%%%%%%% Collision Check here!! %%%%%%%%%%%%%%%%%%%%%%%%
% Here, we discretize the path from node ii-1 to node ii into ellipses with the number of "num_props".
% Then, calculate boundinb boxes (# is "num_props") which surround those ellipses. 
% Finally, run "obstacle_check" for these bounding boxes. 
    num_nbor = numel(nbors_ID);
    issue_flag = true(num_nbor, 1);

    for jj = 1:num_nbor
        node_jj = node(nbors_ID(jj));
        [issue_flag(jj)] = psuedo_obs_check_line_oct(x, P, node_jj, obs_x, R, chi, bound, num_props, prop);

    end

    % find neighboring nodes which don't have any issues.
    In_list_no_issue_ID = nbors_ID(~issue_flag);             

    if isempty(In_list_no_issue_ID) == 1
        nbor_issue = 1; % There is no feasible neighbors
    else
    % Note that the neighbor ID "nbors_ID" is determined by calculating 2-vector
    % norm and Frobenius norm. Thus, this is symmetric distance
    % different with the information cost.
    % 
    % On the other hand, "dist_xk2node_i" is the information
    % cost from node k (existing one) to node ii (new one)             
        val = [node(In_list_no_issue_ID).value].' + dist_xk2node_i(~issue_flag).';
        [val_min, k_min] = min(val);
        parent_ID = In_list_no_issue_ID(k_min);

        P_prime_noissue = P_prime_nei_list(:,:,~issue_flag);
        P_prime_parent2node_i = P_prime_noissue(:,:,k_min);
        
        parent = parent_ID;
        value = val_min;
    end
    
end

end

