function [x, P, obstacle_issue, nearest] = sample_polyshape_check(bound, node, In_list_ID, radius_min, radius, obs_x, cell_size, R, alpha, Metric_id)
% This Function samples and scales a node (x,P) such that hat x is not inside any obstacle
% for fourther info, check the comments at the begining of sample_x_P_randomly

% This works only for 2D cases
x = zeros(1,2);
P = zeros(2);
%% Generate (x,P)
ed = 0;

while ed < radius_min % This checks the distance to the nearest node is greater than radius_min
    
    for jj = 1:2
        
        % Take samples in a bounded region to generate x
        x(jj) = bound(jj).x(1) + (bound(jj).x(2)-bound(jj).x(1))*rand;
    
        % Generate samples for covariance matrix P
        % randi(imax): output integer between 1 and imax
        % P(jj,jj) = bound(jj).P(1) + rand*10^(-randi( abs(log10(bound(jj).P(1)))-abs(log10(bound(jj).P(2))) ));
        
        if jj == 1
            P(jj,jj) = bound(jj).P(1) + rand*10^(-randi( abs(log10(bound(jj).P(1)))-abs(log10(bound(jj).P(2))) ));
        else
            P(jj,jj) = P(1,1) * 10^(-1 + 2*rand(1) );
            if P(jj,jj) > bound(jj).P(2)
                P(jj,jj) = bound(jj).P(2);
            elseif P(jj,jj) < bound(jj).P(1)
                P(jj,jj) = bound(jj).P(1);
            end
        end
       
    end
    
    rad_sam = pi/2 * rand; % uniformly samples the angle 
    Rot = [cos(rad_sam), -sin(rad_sam); sin(rad_sam), cos(rad_sam)];
    P = Rot * P * Rot.'; %This creates the covariance corresponding to an ellipse of the same angle as the initial ellipse
    
    % Finds  nearest: the nearest node in the tree and
    % ed: the distance (Euclidean+Forbenious) from nearest to 
    % generated sample for scaling  
    [nearest,ed] = find_nearest(node, In_list_ID, x, P, R, alpha, Metric_id); 
 end
%% Scales samples (x,P) toward its parent in th tree
    [x, P] = scale_rrt_point(ed,radius,x,node, nearest,P); % scales the sampled point
%% This section checks if the sampled point x is inside any obstacle
obstacle_issue = false;

for ii=1:length(obs_x)
   dist_x= abs((x(1)-obs_x(1,ii)));
   dist_y= abs((x(2)-obs_x(2,ii)));
   
   if dist_x <= cell_size && dist_y <= cell_size  
      obstacle_issue = true;
      break;
   end  
   
end

end

