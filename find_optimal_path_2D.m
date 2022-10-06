function [ path, min_path_leng ] = find_optimal_path_2D( node, In_list_ID, ii, target, N)
% Find the optimal path from the current set of active nodes
% path: the optimal path
% min_path_leng: the length of shortest path
% If there is no path which reaches the target region, path = [] and 
% min_path_leng = -10^10

x_min = target(1,1);
y_min = target(2,1);
x_max = target(1,2);
y_max = target(2,2);

In_list_i_ID = [In_list_ID; ii];

% Used to check whether a bounding box of ellipse is completly in the
% target area.
rec_all = [node(In_list_i_ID).ellipse_rect].';
all_rec = reshape( rec_all, [4, numel(rec_all)/4]).';

% Check whether a bounding box of ellipse is completly in the
% target area.
xmin_check = all_rec(:,1) >= x_min;
xmax_check = (all_rec(:,1) + all_rec(:,3)) <= x_max;
ymin_check = all_rec(:,2) >= y_min;
ymax_check = (all_rec(:,2) + all_rec(:,4)) <= y_max;

% list of nodes inside inside the target region
In_target_check = xmin_check + xmax_check + ymin_check + ymax_check;


% Find the shortest path
In_list_region_ID = In_list_i_ID(In_target_check == 4);
val_all = [node(In_list_region_ID).value].';

[min_path_leng, min_ind] = min(val_all);
if isempty(min_path_leng)
    min_path_leng = -10^10;
end
ind = In_list_region_ID(min_ind); % the node in target region that has the minimum value


% Make the vector which shows the list of nodes that consists the shortest
% path: the order is from the target region toward initial node
path = zeros(N,1);
count = 1;
if ind~=0
    path(count) = ind;
    m = node(ind).parent;
    while m ~= 1
        count = count + 1;
        path(count) = m;
        ind = m;
        m = node(ind).parent;
    end
    path(count + 1) = 1;
end

path(path==0) = [];

