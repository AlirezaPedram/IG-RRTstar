function [obs_x, cell_size] = Map_gen()
% This function generates and plot the map or occupancy grid of the enviroment  

dime=[1; 1]; %dimension of the enviroment in meters 
scale=100; % each pixel shows a 0.01 m by 0.01 meter

cell_size=1/scale;

pixel_num_x = scale * dime(1);
pixel_num_y = scale * dime(2);

map=zeros(pixel_num_x, pixel_num_y);
% map(i,j) = 1 if it is occupied
% map(i,j) = 0 id it is vacant


obs_x=[];

% Definition of Obstacle 1
%  a rectangle with x_min=0.2 m, y_min=0.2 m, \delta x=0.2 m, \delta y= 0.2 m 

for ii=21:40
    for jj=21:40
       map(ii,jj)=1;
       obs_x = [obs_x, [(ii-0.5)* cell_size ; (jj-0.5)* cell_size ]];
    end
end

% Definition of Obstacle 2
%  a rectangle with x_min=0.2 m, y_min=0.6 m, \delta x=0.2 m, \delta y= 0.2 m 
for ii=21:40
    for jj=61:80
       map(ii,jj)=1;
       obs_x = [obs_x, [(ii-0.5)* cell_size ; (jj-0.5)* cell_size ]];
    end
end


% Definition of Obstacle 3
%  a rectangle with x_min=0.5 m, y_min=0.5 m, \delta x=0.2 m, \delta y= 0.2 m 
for ii=51:70
    for jj=51:70
       map(ii,jj)=1;
       obs_x = [obs_x, [(ii-0.5)* cell_size ; (jj-0.5)* cell_size ]];
    end
end


% % % plot environment
% % figure 
% % hold on 
% % box on
% % grid on
% % box on
% % 
% % for ii=1:pixel_num_x
% %     for jj=1:pixel_num_y
% %         if map(ii,jj)==1
% %         fill([(ii-1)/scale  ii/scale ii/scale (ii-1)/scale],[(jj-1)/scale (jj-1)/scale jj/scale jj/scale],'k');
% %         end
% %     end
% % end
% % axis equal
% % xlim([0 dime(1)])
% % ylim([0 dime(2)])


end