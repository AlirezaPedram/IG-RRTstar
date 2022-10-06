function out_flag = boundary_check(ellipse_rect,bound)
% This function checks the collision between the bounding box "ellipe_rect"
% and the boundary of the region "bound"
% returns 0 for no overlap (good)
% returns 1 for the overlap (bad)

%Initial value
out_flag = 1;

x1_min = bound(1).x(1);
x1_max = bound(1).x(2);
x2_min = bound(2).x(1);
x2_max = bound(2).x(2);
rect = ellipse_rect; % [bot_left-x bot_left-y width height]

if rect(1) > x1_min
    if rect(2) > x2_min
        if rect(1)+rect(3) < x1_max
            if rect(2)+rect(4) < x2_max
                out_flag = 0;
            end
        end
    end
end