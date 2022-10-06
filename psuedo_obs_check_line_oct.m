function issue_flag = psuedo_obs_check_line_oct(x, P, node_jj, obs_x, R, chi, bound, num_props, prop)
% This function detemines if the transition from the node_jj to x, P is
% collison-free or not
% 0 (=false) means the transition is collision free
% 1 (=true) means collision happens
% Inputs of this function are
% x: the newly sampled point
% node_jj: The existing nodes information (See the beginning of "main.m for more information")
% obstacle_edge: The information of obstacles
% R: noise power
% chi: The scalar value which determines the confidence bound
% bound: This define the region of the enviroment


%% Initialize Issue_flag
issue_flag = false;
%issue_boundary = true;

%% check losslessness 
not_lossless = check_lossless( node_jj.x, node_jj.P, x, P, R );
if not_lossless == true
    issue_flag = true;
    return;
end
%%
 
% initial and final position of connecting line segment
x0 = node_jj.x;
xF = x;
 
P0 = node_jj.P;
 


%% Check collisions by using the inscribing octagon or rectangle at intermediate points
move_vec = (xF - x0)/(num_props+1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create bounding boxes for eiilpses with the number of "num_props"
for kk = 1:num_props+1
    prop(kk).x = x0 + move_vec*kk;
    if kk > 1
        dt = norm(prop(kk).x-prop(kk-1).x);
        prop(kk).P = prop(kk-1).P + R*dt;
    else
        dt = norm(prop(kk).x-x0);
        prop(kk).P = P0 + R*dt;
    end
      [prop(kk).ra,prop(kk).rb,prop(kk).ang,prop(kk).ellipse_rect] = error_ellipse(prop(kk).x,prop(kk).P,chi);    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for kk=1:length(prop)
    
        if obstacle_check_grid_matrix(prop(kk).x, prop(kk).P, chi, obs_x)
            issue_flag = true;
            return
        end
    
    rect = prop(kk).ellipse_rect;
    % In case the boundary is a rectangle that stand upright, then
    % rectangle is enough to check collision with boundaries.
    if rect(1) < bound(1).x(1) || rect(2) < bound(2).x(1) || rect(1)+rect(3) > bound(1).x(2) || rect(2)+rect(4) > bound(2).x(2)
        issue_flag = true;
        return
    end


end



end


