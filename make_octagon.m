function oct_edges = make_octagon(pos, ra, rb, phi, rect)
% This function generates an inscribing (enclosing) octagon for gievn error ellipse
% This function is used for collision checking

Rot = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
box = [ra, rb; -ra, rb; -ra, -rb; ra, -rb];
rot_box = box * Rot + repmat(pos, [4,1]);


[~, y_mx_e] = max(rot_box(:,2));
rot_box2 = [rot_box; rot_box];

diff_box = diff( rot_box2( y_mx_e:y_mx_e+2,: ) );
slope_X = diff_box(:,1) ./ diff_box(:,2);
slope_Y = 1./slope_X;

% From Ymax, Counter Clock wise
x0y0_list = rot_box2(y_mx_e:y_mx_e+3, :) + repmat(pos, [4,1]);
% box_y_list = [pos(2)+y_box; pos(2)-y_box; pos(2)-y_box; pos(2)+y_box ];
box_y_list = [rect(2) + rect(4); rect(2); rect(2); rect(2) + rect(4)];

x_cross_pts = x0y0_list(:,1) + [slope_X; slope_X] .* (box_y_list - x0y0_list(:,2)); 
box_y_Cross = [x_cross_pts, box_y_list];
% box_x_list = [pos(1)-x_box; pos(1)-x_box; pos(1)+x_box; pos(1)+x_box];
box_x_list = [rect(1); rect(1); rect(1) + rect(3); rect(1) + rect(3)];
    
y_cross_pts = x0y0_list(:,2) + [slope_Y; slope_Y] .* (box_x_list - x0y0_list(:,1)); 
box_x_Cross = [box_x_list, y_cross_pts];

sm_octagon = [box_y_Cross(1,:); box_x_Cross(1:2,:); box_y_Cross(2:3,:);...
    box_x_Cross(3:4,:); box_y_Cross(4,:)];
oct_edges_pre = [sm_octagon; sm_octagon(1,:)];
oct_edges_pre2 = [oct_edges_pre, oct_edges_pre([2:end,1],:)];
oct_edges_pre2(end,:) = [];

oct_edges = [oct_edges_pre2(:,1:2), oct_edges_pre2(:,3:4) - oct_edges_pre2(:,1:2)];
end