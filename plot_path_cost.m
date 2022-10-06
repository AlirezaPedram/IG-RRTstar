close all
% Function to plot the path length
% X-axis: Number of steps
% Y-axis: The lengh of the optimal path at that step
% Should be ran after main
 
 
fig_path_cost = figure;
clf(fig_path_cost);
set(fig_path_cost,'color','white');

scatter(1:N, min_path_data,'r','filled')
xlim([0 N])
ylim([0 max(min_path_data)])
grid on
box on
set(gca, 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('Number of Nodes', 'FontSize', 22, 'interpreter', 'latex');
ylabel('Path Cost, $c(\sigma)$', 'FontSize', 22, 'interpreter', 'latex');

set(gca,'LineWidth',1)
ax = gca;
ax.LineWidth = 1;
