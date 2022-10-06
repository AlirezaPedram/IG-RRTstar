function obstacle_issue = obstacle_check_grid_matrix(x, P, chi, obs_x)
%check if (x,P) is in collsion with obstacle cells

% since cells has an nonzero_length we add a heuristic safety margin
% epsilon
epsilon=0.5; 

Grid2Cent = (obs_x.' - x);
Grid2Cent_P = Grid2Cent * inv(P);
level_vec = sum(Grid2Cent_P .* Grid2Cent, 2);

obstacle_issue = any(level_vec <= chi+epsilon);

end