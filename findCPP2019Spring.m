function [CPP_t, path_err, angle_err] = findCPP2019Spring(x, y, theta, points, path_fwd)
%FINDCPP2019SPRING Summary of this function goes here
%   Detailed explanation goes here
smallest_dist = 10000;
closest_index = 1;
for index = 1:length(points)
    this_dist_sq = (points(1,index) - x)^2 + (points(2,index) - y)^2;
    if this_dist_sq < smallest_dist
       smallest_dist = this_dist_sq;
       closest_index = index;
    end    
end
CPP_t = closest_index;
CPP_t_safe = max(2, closest_index);
%get path angle at point
path_angle = angleDiff(atan2(points(2, CPP_t_safe) - points(2,CPP_t_safe-1),...
                             points(1, CPP_t_safe) - points(1,CPP_t_safe-1)), -0*pi*(path_fwd==0));
angle_to_robot = atan2(y-points(2, CPP_t_safe), x - points(1, CPP_t_safe));
angle_err = angleDiff(path_angle, theta);
if angleDiff(angle_to_robot, path_angle) > 0
    path_sign = 1.0;
else
    path_sign = -1.0;
end
path_err = path_sign * smallest_dist;

end

