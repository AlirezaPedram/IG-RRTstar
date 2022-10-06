clear all
close all
clc

N = 2000; % Maximum number of nodes


for dum=0.1:0.1:0.1
    clearvars -except dum N

%% Saving parameters
% Variables used for save data
save_node_parent = cell(N,1);
save_node_children = cell(N,1);
save_node_children_num = cell(N,1);
save_node_value = cell(N,1);
save_node_removed = cell(N,1);
save_In_list_ID_i = cell(N,1);
save_children_temp = zeros(N*100,1);

% How often we save the path 
save_freq = 1;
saver(1:N/save_freq) = struct('path', [], 'node',[]);
min_path_data = zeros(N,1);
min_path = cell(N,1);


%%  Definition of the tree structure

% Just setting initial value 
ini_st = -100; % initial value for unsampled nodes
ini_P = diag([-100 5]); % initial covariance for unsampled nodes
ini_value = 5000; % initial cost for unsampled nodes

% Initilize node with the value which will not be outputted by algorithm
node(1:N) = struct('x',ini_st*ones(1,2), 'P', ini_P, 'parent', ini_st,...
    'children',int16( ini_st*ones( ceil(N/3) ,1) ), 'children_num', 0,...
    'value', ini_value, 'ra', ini_st, 'rb', ini_st, 'ang', ini_st,...
    'ellipse_rect', ini_st*ones(1,4), 'removed', true);

% node.x: The position (2-D) of the node   (1*2 vector)
% node.P: The covariance (2-D) of the node (2*2 matrix)
% node.parent: The ID of the parent node of each node 
%     (Since each node should have only one parent, this is a scalar value)
% node.children: The ID of the children nodes for each node   (vector)
%     (The size of node.children is determined under expectation "the maximum
%       number of children will not surpass the determined size here" )
% node.children_num: This expresses how many nodes has been assigned to 
%     the children of each node.   (scalar value)
%     ####### CAUTION ######## 
%     This value does not corresponds to the actual number of 
%     the children nodes, since the children is deleted in the rewired process.
%     The actual number is obtained by  
%     "A = node(parent_ID).children;  A(A==ini_st) = []; length( A )"
%     ########################
% node.value: The pathlength from the initial node to each node (scalar value)
% node.ra: The length of major axis of ellipse  (scalar value)
% node.rb: The length of minor axis of ellipse  (scalar value)
% node.ang: The rotation angle of the ellipse (range is from 0 to 2*pi) (scalar value)
% node.ellipse_rect: A bounding box which surrouds the ellipse 
%        [bottom-left-x bottom-left-y width height]  ([1, 4] matrix)
% node.removed: If this value is true, the node is not under consideration.
%     For example, if branch and bound delete node i, node.removed  for node i
%     becomes true. (Boolean)
% The size of node.children is determined under expectation "the maximum
% number of children will not surpass the determined size here like N/3" 

%% Branch-and-bound parameters

%Every bnb_val step, the branch and bound deletes the unnecessary nodes.
bnb_val = 10;
% To save the data
remove_ID = cell(N/bnb_val,1);

%% Variables used for the RRT*

% neighboring (=rewiring) radius computation
radius = 0.5; %neiborhood and rewirimg radius
radius_min = inv(realmax); %minimum radius
% dimension of the space (Current version only works with "dim = 2")
dim=2; % dimension of the problem
miu_X_free=1;
zeta_d= pi; % volume of unit ball in dim-dimensional space  
gamma_star = (2+2/dim)^(1/dim)* (miu_X_free/zeta_d)*(1/dim);


% Weight on information cost
alpha = dum;

% The gain of noise (In the paper, we denote as "W")
R = (1/1000)*eye(2);

% 90 percent confidence bound
chi = chi2inv(0.9,2);

%% Enviroment definition and Properties

% Current enviroment is  " Quad enviroment" 
        
        % define obstacle as a set of edges 
        % each edge is defined by: start point, end point, slope, and Y_axis
        %definition of the map
        [obs_x, cell_size] = Map_gen();
        
        % Target(final) area [xmin, xmax; ymin, ymax]
        target = [0.8, 0.9; 0, 0.1]; 

        % Path planning area
        bound(1).x = [0,1]; % X direction
        bound(2).x = [0,1]; % Y direction
        
        % The acceptable range for the eigenvalues of the sampled covariance matrix
        bound(1).P = [10^-9,10^-1]; 
        bound(2).P = [10^-9,10^-1];
        
        % The position of the initial node
        node(1).x = [0.1, 0.9];

%% Metric selection

 Metric_id = 1; % If Entropy is used

% %  Metric_id = 2; % If Wasserstein distance is used

% %   Metric_id = 3; % If Hellinger distance is used

%% The setting for initial node
node(1).P = [bound(1).P(1),0;0,bound(2).P(1)];
node(1).parent = 0;
node(1).value = 0;

[node(1).ra,node(1).rb,node(1).ang,node(1).ellipse_rect] = error_ellipse(node(1).x, node(1).P, chi);
% [ra=major axis, rb=minor axis, ang= rotation angle , rect=bounding box] 
% = error_ellipse(x= 2D position of the ellipse, P = covariance , chi= confidence level)

node(1).removed = false; % Node 1 is in the tree and active
min_path_leng{1} = -10^10; % No path to target region is found initially

%% Parameters for collision checking in transition between nodes

% How many intermediate ellipses to use for collision check
num_props = 10;

% Definition of intermediate ellipses 
prop(1:num_props) = struct('x', ini_st*ones(1,2), 'P', ini_st*eye(2), 'ra', ini_st,...
    "rb", ini_st, 'ang', ini_st, 'ellipse_rect', zeros(1,4)); %, 'flag', false);

% Initialize "prop", which have following structure
% prop.x: The position (2-D) of the node  (1*2 vector)
% prop.P: The covariance (2-D) of the node  (2*2 matrix)
% prop.ra: The length of major axis of ellipse  (scalar)
% prop.rb: The length of minor axis of ellipse  (scalar)
% prop.ang: The rotation angle of the ellipse   (range is from 0 to 2*pi)
% prop.ellipse_rect: A bounding box which surrouds the ellipse
%        [bottom-left-x bottom-left-y width height]
% prop.ellipse_rect: Used for collision checking (Boolean)

%% Main Algorithm

% Counter used for saving the data
save_counter = 0;
tic
% main loop of IG RRT* Algorithm
for ii=2:N
    
    % New node ii becomes active and will be added to the list of nodes
    node(ii).removed = false;
    In_list_ID = find(~[node(1:ii-1).removed]).'; % A list contains IDs of active nodes 
    num_list = numel(In_list_ID); % compute the number of active nodes
    
    % Compute neighboring (=rewiring) radius 
    dummy= 50 * gamma_star*(log(num_list+1)/(num_list+1))^(1/dim);
    r_attempt= min(radius,dummy);
    %r_attempt = radius;
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Inserting a new node to the tree %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
        nbor_issue = 1;
        while nbor_issue ~= 0
            % Sample and scale (x,P) in free space
            % nearest is the nearest node on the tree to (x,P)
            [x, P, ra, rb, ang, ellipse_rect, nearest] = sample_x_P_randomly(bound, node,In_list_ID, radius_min, r_attempt, chi, obs_x, cell_size, R, alpha, Metric_id);
       
            % Parent and Value search: 
            % nbor_issue = true means no parent is found
            % P_prime_parent2node_i is the minimizer of the optimization
            % Value is the total cost
            % P_prime_parent2node_i is loss-less modification of P 
            [parent, value, nbor_issue, P_prime_parent2node_i] =...
                find_parent(x, P,node,R,alpha, r_attempt, obs_x, chi, bound, num_props, prop, In_list_ID, nearest, Metric_id);
        end
       
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Insert the sampled node's information in node(ii) 
        node(ii).x = x;
        node(ii).P = P;
        node(ii).ellipse_rect = ellipse_rect;
        node(ii).value = value;
        node(ii).parent = parent;
        node(ii).ra = ra;
        node(ii).rb = rb;
        node(ii).ang = ang;
        node(parent).children_num = node(parent).children_num + 1;
        node(parent).children( node(parent).children_num ) = int16(ii);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%% Rewiring %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % finds the neighboring nodes (nodes within r_nbor radius of newly-added node) for rewiring 
        Is_only_neibor_ID = true;
        
        [nbors_rw_ID, ~, ~, ~] = find_neighbors( x, P, node, R, alpha, r_attempt, In_list_ID, Is_only_neibor_ID, false, Metric_id);
        
        % find collision-free neighboring nodes 
        num_rw_nbor = numel(nbors_rw_ID);
        issue_flag2 = true(num_rw_nbor, 1);
        
        for jj = 1:num_rw_nbor
            node_jj = node(nbors_rw_ID(jj));
            % check collision 
            issue_flag2(jj) = psuedo_obs_check_line2_oct(node(ii), node_jj, obs_x, R, chi, bound, num_props, prop);
        end
        % list of collision-free neighboring nodes 
         nbors_rw_no_issue_ID = nbors_rw_ID(~issue_flag2);
         
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Are there any negiboring nodes?
        if ~isempty(nbors_rw_no_issue_ID)
            
            % reshaping the data
            x_nbors = [node(nbors_rw_no_issue_ID).x].';
            P_nbors = [node(nbors_rw_no_issue_ID).P];
            x_nbors_mat = reshape( x_nbors, [dim, numel(x_nbors)/dim]);
            
            % total cost = Euclidean cost+ Info cost from node ii to k
            [dist_node_i2xk, dInf_i2xk, P_prime_i2xk_list] = dist_ig_mat2(node(ii).x.', node(ii).P, x_nbors_mat, P_nbors, alpha, R, Metric_id);
            
            % new values of the nodes if they get rewired
            val_new = node(ii).value + dist_node_i2xk.';
            
            % finds the nodes required to be rewired (rewired_better_ID)
            rewired_ID_in_nbors_rw = find(val_new < [node(nbors_rw_no_issue_ID).value].');
            rewired_better_ID = nbors_rw_no_issue_ID(rewired_ID_in_nbors_rw);
            
            
            % Are there any nodes we should rewire?
            if ~isempty(rewired_better_ID)
   
                % ID of the old parents of rewired_better_ID 
                % From their children list we should remove "rewired_better_ID"
                old_parent_ID = [node(rewired_better_ID).parent].'; % old_parent: Vector size: same length with rewired_better_ID

                % Matrix Size (2D): [length( node(1).children ) * length(rewired_better_ID)]
                %mat_parents_rewired_better = [node(old_parent_ID).children];

                % For the node we should rewire
                for k=1:length(rewired_better_ID)
                    
                    mat_parents_rewired_better = [node(old_parent_ID).children];
                    
                    % log in the new parent and the new value
                    node(rewired_better_ID(k)).parent = ii;
                    node(rewired_better_ID(k)).value = val_new(rewired_ID_in_nbors_rw(k));
                    
                    
                    % update node ii children list 
                    node(ii).children_num = node(ii).children_num + 1;
                    node(ii).children( node(ii).children_num ) = int16(rewired_better_ID(k));
                    
                    
                    % modify the children list of the old parents
                    % Overwrite "rewired_better_ID(k)" in the old parent's list of children with "ini_st"
                    mat_parents_rewired_better([node(old_parent_ID(k)).children] == rewired_better_ID(k), k) = int16( ini_st );
                    node(old_parent_ID(k)).children = mat_parents_rewired_better(:, k);
                    
                    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% updating the descendants of rewired nodes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
                    % All direct children of the rewired node k
                    % We will update the information of these children in
                    % the following for loop
                    
                    direct_child_k_ID = [node(rewired_better_ID(k)).children].';
                    direct_child_k_ID( direct_child_k_ID == int16(ini_st) ) = [];
                    num_child = length(direct_child_k_ID); % number of direct children
                    
                    % All descendants ID whose path contains the rewired node k
                    descendants_k_ID = zeros(N,1);
                    descendants_k_ID(1:num_child) = direct_child_k_ID;
                    
                    count = num_child;
                    child = direct_child_k_ID;% list of children
                    
                    
                    % find all desecndant in the following loop
                    while num_child > 0
                        
                        next_child_all = zeros(N,1);
                        cnt_child = 0;
                        
                        for cnt = 1:num_child
                            
                            next_child = [node( child(cnt) ).children].';
                            next_child(next_child == ini_st) = []; 
                            num_next_chi = length(next_child);

                            % Indirect children of node k is accumulated here
                            descendants_k_ID(count+1:count+num_next_chi) = next_child;
                            count = count + num_next_chi;

                            next_child_all(cnt_child+1:cnt_child+length(next_child)) = next_child;
                            cnt_child = cnt_child+length(next_child);
                        end
                        
                        next_child_all(next_child_all == 0) = [];
                        child = next_child_all;
                        num_child = length(child);   
                    end
                    
                    descendants_k_ID(descendants_k_ID == 0) = [];
                    num_descendants_k = length(descendants_k_ID); % list of all descendants
                    
                    % For all nodes which paths were changed, update their path length 
                    % This updates do not contain "rewired_better_ID(k)",
                    % but all the nodes after "rewired_better_ID(k)".
                    % This part also contains lossless refinement
                    for cnt=1:num_descendants_k
                        % From 'm' to 'n'.
                        n=descendants_k_ID(cnt);
                        m=node(n).parent;
                        xn=node(n).x;
                        Pn=node(n).P;
                        xm=node(m).x;
                        Pm=node(m).P;
                        
                        % log in the new value 
                        [dmn, dInf_mn, P_prime_desc] = dist_ig_mat(xm.',Pm,xn.',Pn, alpha, R, Metric_id);
                        node(n).value=node(m).value+dmn;
                        
                    end                    
                end
            end            
        end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    % Try to find the optimal path from the current set of nodes
    % If there is no path which reaches the target region, path = [].
    [path, min_path_leng] = find_optimal_path_2D( node, In_list_ID, ii, target, N);
    min_path_data(ii) = min_path_leng; % length of the shortest path
    min_path{ii} = path; % shortest at step ii  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Branch-and-bound %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Run branch and bound every "bnb_val"
    if rem(ii,bnb_val) == 0
        if ~isempty(path) 
            [node, remove_ID{ii/bnb_val}] = branch_and_bound_2D(node,path,In_list_ID, alpha,R, ii, ini_st, dim, Metric_id); 
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % run-time output in command window  
    clc;
    disp(ii) % print the step
    disp(~isempty(path))   % show if a path is found
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % saving parameters
    % saving at save_freq frequency
    
    if rem(ii,save_freq) == 0
        save_counter = save_counter + 1;
        saver(save_counter).path = path;
        saver(save_counter).node = ii;
    end
    
    % save the node list 
    In_list_ID_i = find(~[node(1:ii).removed]).';
    save_In_list_ID_i{ii} = In_list_ID_i;
    save_node_parent{ii} = [node([In_list_ID_i]).parent];
    
    % Save the necessary data
    cnt_save = 0;
    for kk = 1:numel(In_list_ID_i)
        children_for_save = node(In_list_ID_i(kk)).children;
        children_for_save(children_for_save == ini_st) = [];
        save_children_temp(cnt_save+1:cnt_save+numel(children_for_save),1) = children_for_save;
        cnt_save = cnt_save + numel(children_for_save);
    end
    
    save_children_temp(save_children_temp==0) = [];
    save_node_children{ii} = save_children_temp;
    save_node_children_num{ii} = [node(In_list_ID_i).children_num];
    save_node_value{ii} = [node(In_list_ID_i).value];
    save_node_removed{ii} = [node(In_list_ID_i).removed];
    
end
%% Output Data
%%%%%%%%%%%%%% All data should be saved here %%%%%%%%%%%%%%%
% The file name used for save the data
% Data is saved in "data" folder
% Name includes N, alpha value, safety percentage 
savename = ['data/Multi_N_Ent_',num2str(N),'_alpha_',num2str(alpha),'_safety_', num2str(0.9)];
savename(savename=='.') = [];
save(savename)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
timeElapsed = toc