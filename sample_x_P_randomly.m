function [x, P, ra, rb, ang, ellipse_rect, nearest] = sample_x_P_randomly(bound,node,In_list_ID,radius_min,radius, chi, obs_x, cell_size, R, alpha, Metric_id)
% This funtion samples a node (x,P)
% Sampling (x,P) is performed in two steps
% Step 1: a "scaled" (x,P) is sought such that x is not inside any obstacle
% Step 1 is performed by sample_polyshape_check
% Step 2: check the confidence ellipse associated with (x,P) is colliding
% with any obstacle or outer boundary 
% if the collsion is detected, step 2 reduces P by factor gamma until a
% collsion free (x,P) is obtained

gamma=0.5; % rescaling covariance factor

%% Step 1: Samples until it finds a "scaled" (x,P) such that x is outside of all obstacle regions 
obstacle_issue = 1;

while  obstacle_issue ~= 0
[x, P, obstacle_issue, nearest] = sample_polyshape_check(bound,node,In_list_ID,radius_min, radius, obs_x, cell_size, R, alpha, Metric_id);
end
% nearest is parent of the sampled, sclaed (x,P)

 %% Step 2: checks for instersection with outer boundry and obstacle regions

 
%num_obs_edge_ini = length(obstacle_edge); % number of edges in obstacles
%obs_st = reshape([obstacle_edge(:).start], [2, num_obs_edge_ini]).';
%obs_end = reshape([obstacle_edge(:).end], [2, num_obs_edge_ini]).';

while true 

   [ra,rb,ang,ellipse_rect] = error_ellipse(x,P,chi); % compute the bounding box of (x,P)
 
   %%%%%%%%% Check Collision with region's outer boundary%%%%%%%%%%%%%%%%
    boundary_issue = boundary_check(ellipse_rect, bound);
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %%%%%%%%% Check Collision with remaining obstacle edges %%%%%%%%%%%%%%
        obstacle_issue = obstacle_check_grid_matrix(x, P, chi, obs_x);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if boundary_issue == false && obstacle_issue == false
        break
    end
    
P = gamma * P; % shrink the covarinace by factor gamma
end





% % % while true
% % %     
% % % %   Initialize
% % %     obs_edge_delete_list = zeros(num_obs_edge_ini, 1);
% % %  
% % %     obstacle_edge_list = [obs_st, obs_end];
% % %     
% % %     [ra,rb,ang,ellipse_rect] = error_ellipse(x,P,chi); % compute the bounding box of (x,P) 
% % %     
% % %     %%%%%%%%%%%%%%%%% Pre-check algorithm %%%%%%%%%%%%%%%%%%%%
% % %     
% % %     % here, we perform a pre-check to reduce number of collison checking
% % %     % and  thus the run time
% % %     % This section checks if the distance between x and an edge is greater
% % %     % than ra which means collision is impossible, and checks if the
% % %     % distance is lower than rb which means collision is detected
% % %     
% % %     for k = 1:num_obs_edge_ini
% % %         dist_Cnt2ObsEdge = dist_point2lineseg(x, obstacle_edge(k).start, obstacle_edge(k).end);
% % %         if dist_Cnt2ObsEdge < rb
% % %             % Collision happens, get out of this for-loop and shrink ellipse
% % %             obstacle_issue = true;
% % %             break
% % %         elseif dist_Cnt2ObsEdge > ra
% % %             % Delete edges that would not cause collision
% % %             obs_edge_delete_list(k) = k;
% % %         end
% % %     end
% % %     obstacle_edge_list( (obs_edge_delete_list ~= 0), : ) = [];
% % %     num_obs_edge = size(obstacle_edge_list, 1);
% % %     
% % %     %%%%%%%%% Check Collision with remaining obstacle edges %%%%%%%%%%%%%%
% % %     if num_obs_edge == 0
% % %         obstacle_issue = false;
% % %     elseif num_obs_edge > 0 && obstacle_issue == false
% % %         obstacle_issue = obstacle_check_line_oct(x, ra, rb, ang, ellipse_rect, obstacle_edge_list);
% % %     end
% % %     
% % %     %%%%%%%%% Check Collision with region's outer boundary%%%%%%%%%%%%%%%%
% % %     boundary_issue = boundary_check(ellipse_rect, bound);
% % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % %     
% % %     if boundary_issue == false && obstacle_issue == false
% % %         break
% % %     end
% % %     
% % % P = gamma * P; % shrink the covarinace by factor gamma
% % % end
% % % 
% % % 
% % % end
% % % 
% % % % dist_point2lineseg(x, edge.start, edge.end) returns the distence
% % % % between point x and the line segment between edge.start and edge.end
% % % function dist_pt2LineSeg = dist_point2lineseg(pt, line_St, line_Ed)
% % %     line_seg = line_Ed - line_St;
% % %     lineSt2_pt = pt - line_St;
% % %     
% % %     cross_twoLine = dot(lineSt2_pt, line_seg);
% % %     cross_lineSeg = dot(line_seg, line_seg);
% % %     
% % %     if cross_twoLine <= 0
% % %         dist_pt2LineSeg = norm(lineSt2_pt);
% % %     elseif cross_twoLine < cross_lineSeg
% % %         d2 = cross_twoLine / norm(line_seg);
% % %         dist_pt2LineSeg = sqrt( norm(lineSt2_pt)^2 - d2^2 );
% % %     else
% % %         lineEd2_pt = pt - line_Ed;
% % %         dist_pt2LineSeg = norm(lineEd2_pt);
% % %     end
% % % end
% % % 
% % % % obstacle_check_line_oct checks the collision between boundin box/octagon
% % % % and a list of edges
% % % function issue_flag = obstacle_check_line_oct(x, ra, rb, ang, rect, obstacle_edge)
% % %     % For more information, please check this website.
% % %     % https://stackoverflow.com/questions/4977491/determining-if-two-line-segments-intersect/4977569#4977569
% % %     
% % % issue_flag = false;
% % % % Do you check collision with box or octagon?
% % % is_oct = true;
% % % 
% % % 
% % % if mod(ang, pi/2) < pi/180*10
% % %     is_oct = false;
% % % end
% % % 
% % % 
% % % if is_oct == true
% % %     bound_para = make_octagon(x, ra, rb, ang, rect); % build octagon 
% % % else
% % %     % reshape bounding box
% % %     vertix = [rect(1), rect(2), rect(1)+rect(3), rect(2);... % bottom-horizontal
% % %             rect(1)+rect(3), rect(2), rect(1)+rect(3), rect(2)+rect(4);... % right-vertical
% % %             rect(1), rect(2)+rect(4), rect(1)+rect(3), rect(2)+rect(4);... % top-horizontal
% % %             rect(1), rect(2), rect(1), rect(2)+rect(4)]; % left-vertical
% % %         bound_para = [vertix(:,1:2), vertix(:,3:4) - vertix(:,1:2)];
% % % end
% % %     
% % % 
% % % for bb = 1:size(obstacle_edge, 1)
% % %     % [obs_st1, obs_end1; obs_st2, obs_end2; ...]
% % %     obs_edge = [obstacle_edge(bb, 1:2), ...
% % %             obstacle_edge(bb, 3:4) - obstacle_edge(bb, 1:2)];
% % %     
% % %     % for each edge of ellipse's bounding box/octagon
% % %     for j = 1:size(bound_para,1)
% % %         is_cross = Is_two_lineseg_cross(bound_para(j,:), obs_edge);
% % %         if is_cross == true
% % %             issue_flag = true;
% % %             return
% % %         end
% % %     end
% % %     
% % %     % If the edge is in the box
% % %     % As we don't now which direction the path from start to end, this is
% % %     if rect(1) <= obstacle_edge(bb, 1) && rect(1)+rect(3) >= obstacle_edge(bb, 3) && rect(2) <= obstacle_edge(bb, 2) && rect(2)+rect(4) >= obstacle_edge(bb, 4)
% % %         issue_flag = true;
% % %         return
% % %     end
% % %     if rect(1) <= obstacle_edge(bb, 3) && rect(1)+rect(3) >= obstacle_edge(bb, 1) && rect(2) <= obstacle_edge(bb, 4) && rect(2)+rect(4) >= obstacle_edge(bb, 2)
% % %         issue_flag = true;
% % %         return
% % %     end
% % %     
% % % end

end

