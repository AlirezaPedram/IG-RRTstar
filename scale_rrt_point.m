function [x, P] = scale_rrt_point(dist,radius,x,node,nbor, P)
% Check if the node (x,P) is within the predefined distance (radius) of the 
% neighbor node "nbor"
% if not it is not whithin "raduis" distance, this function scales the node
% dist: initial distance to "nbor"

if dist> radius
    
    % % scale the Euclidean location (no covariance change),  
    nbor_x = node(nbor).x;
    dx = (x - nbor_x) .* (radius/dist);
    x = nbor_x + dx;
    
    % %  scale covariance until it meets the info region
    nbor_P = node(nbor).P;
    
    dP = (P - nbor_P) .* (radius/dist);
 
    P = nbor_P + dP;
end
end 
