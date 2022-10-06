close all
% This function plots the enviroment and the optimal path
% You need to run this codes after loading mat file or right after you
% finished run the simulation

% Log in the data
 %load('data/Multi_N5000_alpha_3_safety_09')
load('data/Multi_N_Ent_2000_alpha_01_safety_09')
obstacle_edge = obstacle_quad();
% start the figure
figure
hold on

axis equal
xlim([bound(1).x(1) bound(1).x(2)]);
ylim([bound(2).x(1) bound(2).x(2)]);
hold on

% colors used in the figure
color_b = [0    0.4470    0.7410];
color_p = [0.4940    0.1840    0.5560];
% color used to fill the obstacles
%color_fill = [1, 0, 0]; 
color_fill = [187/255,139/255,172/255];

edges = [obstacle_edge(:).start];
edges_re = reshape(edges,[2,numel(edges)/2]);
% You need to designate the number of vertices of each obstacle here
% The first element should be 0
num_vert = [0, 4, 4, 4];


% goal region patch : green
x_patch = [target(1,1) target(1,2) target(1,2) target(1,1)];
y_patch = [target(2,1) target(2,1) target(2,2) target(2,2)];
patch(x_patch,y_patch, [0.8 1 0.8])

%% plot obstacles

ini = 0;
for kk = 1:length(num_vert)-1
    ini = ini + num_vert(kk);
    las = ini + num_vert(kk+1);
    fill( edges_re(1, ini+1:las).' , edges_re(2, ini+1:las).' ,color_fill)
end
for kk = 1:length(obstacle_edge)
    fill([obstacle_edge(kk).start(1) obstacle_edge(kk).end(1)], [obstacle_edge(kk).start(2) obstacle_edge(kk).end(2)],'k','LineWidth',0.5)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If you want to plot all edges, please uncomment this for-loop
 for kk = 2:length(save_In_list_ID_i{N})
              
        %plot(vertex(ii).x(1), vertex(ii).x(2), 'kx')   
     jj = save_In_list_ID_i{N}(kk);
     parent = save_node_parent{N}(kk);
     plot([node(jj).x(1) node(parent).x(1)],[node(jj).x(2) node(parent).x(2)],'color', [0.7, 0.7, 0.7],'LineWidth',0.4);
   
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% plot optimal path and confidence ellipses
theta_grid = linspace(0,2*pi);
% Plot ellipses connecting two nodes
for jj = length(saver(end-1).path):-1:2
    x0 = node(saver(end-1).path(jj)).x;
    xF = node(saver(end-1).path(jj-1)).x;
    P0 = node(saver(end-1).path(jj)).P;
    PF = node(saver(end-1).path(jj-1)).P;
    theta_grid = linspace(0,2*pi);
    dx = norm(x0-xF);

    move_vec = (xF - x0)/(num_props+1);

    for zz = 1:num_props+1
        prop(zz).x = x0 + move_vec*zz;
        if zz > 1
            dt = norm(prop(zz).x-prop(zz-1).x);
            prop(zz).P = prop(zz-1).P + R*dt;
        else
            dt = norm(prop(zz).x-x0);
            prop(zz).P = P0 + R*dt;
        end
        [prop(zz).ra,prop(zz).rb,prop(zz).ang,prop(zz).ellipse_rect] = error_ellipse(prop(zz).x,prop(zz).P,chi);
        ellipse_x_r  = prop(zz).ra*cos( theta_grid );
        ellipse_y_r  = prop(zz).rb*sin( theta_grid );
        Rot = [ cos(prop(zz).ang) sin(prop(zz).ang); -sin(prop(zz).ang) cos(prop(zz).ang) ];
        r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
        plot(r_ellipse(:,1) + prop(zz).x(1),r_ellipse(:,2) + prop(zz).x(2), 'color', color_b)
        hold on;
    end
end
%%
% Plot initial node with red dots
plot(node(1).x(1),node(1).x(2),'ro','MarkerSize',6,'MarkerFaceColor','r');
%%
% Plot each node
for jj = 1:length(saver(end-1).path)-1
    plot([node(saver(end-1).path(jj)).x(1) node(saver(end-1).path(jj+1)).x(1)],[node(saver(end-1).path(jj)).x(2) node(saver(end-1).path(jj+1)).x(2)],'k','LineWidth',2);
    plot(node(saver(end-1).path(jj)).x(1),node(saver(end-1).path(jj)).x(2),'ko','MarkerSize',4,'MarkerFaceColor','k');
    ellipse_x_r  = node(saver(end-1).path(jj)).ra*cos( theta_grid );
    ellipse_y_r  = node(saver(end-1).path(jj)).rb*sin( theta_grid );
    Rot = [ cos(node(saver(end-1).path(jj)).ang) sin(node(saver(end-1).path(jj)).ang); -sin(node(saver(end-1).path(jj)).ang) cos(node(saver(end-1).path(jj)).ang) ];
    r_ellipse = [ellipse_x_r;ellipse_y_r]' * Rot;
    plot(r_ellipse(:,1) + node(saver(end-1).path(jj)).x(1),r_ellipse(:,2) + node(saver(end-1).path(jj)).x(2),'k-','LineWidth',1.5)
end


%%
% Plot boundary of region and target region
rectangle('Position',[bound(1).x(1), bound(2).x(1), bound(1).x(2) - bound(1).x(1),  bound(2).x(2)- bound(2).x(1)],'EdgeColor',color_fill,'LineWidth',1.2)
rectangle('Position',[target(1,1) target(2,1) target(1,2)-target(1,1) target(2,2)-target(2,1)],'EdgeColor',[0 0.5 0],'LineWidth',1.5)

xlabel("Location X [m]")
ylabel("Location Y [m]")

% For multi
set(gca, 'FontName', 'Arial', 'FontSize', 22)
ylim([bound(2).x(1) bound(2).x(2)]);
set(gca,'color','white');
grid on
set(gca,'LineWidth',1)
ax = gca;
ax.LineWidth = 1;


fig_f = gcf;
% The size and position of figure
% For multi
fig_f.Position = [650 100 550 512];
xticks([0 0.2 0.4 0.6 0.8 1])

% Put "Start" and "Goal" text here
% For multi alpha 0 0.1

%text(0.06,0.45, 'Start','FontSize', 22);
%text(0.71, 0.08, 'Goal','FontSize', 22);

text(0.04,0.95, 'Start','FontSize', 22);
text(0.61, 0.06, 'Goal','color',[0 0.5 0],'FontSize', 22);

