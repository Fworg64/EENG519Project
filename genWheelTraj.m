%generate wheel trajectory
%given a wheel jerk input, gives the wheel velocity output
function [leftVels, rightVels, leftJerk, rightJerk] = genWheelTraj(time_to_solve)

left_u  = @(t) ((t<=6).*((t<=2).*.8.*sin(2*pi*(t/2)) -...
                 (t>=4).*.8.*sin(2*pi*(t/2))))';
right_u = @(t) ((t<=8).*((t<=4).*.8.*sin(2*pi*(t/2)) -...
                 (t>=6).*.8.*sin(2*pi*(t/2))))';
U = [.5*left_u(time_to_solve), .5*right_u(time_to_solve)];
 
 p = ss([0, 1; 0, 0], [0;1], eye(2,2), zeros(2,1));
 pd = c2d(p, time_to_solve(2) - time_to_solve(1));
 
 state1_rec = zeros(2, length(time_to_solve));
 state2_rec = zeros(2, length(time_to_solve));
 for index = 2:length(time_to_solve)
     state1_rec(:,index) = pd.A*state1_rec(:,index-1) + pd.B*U(index,1);
     state2_rec(:,index) = pd.A*state2_rec(:,index-1) + pd.B*U(index,2);
 end
 
 leftVels  = state1_rec(1, :);
 rightVels = state2_rec(1,:);
 
 leftJerk = U(:,1);
 rightJerk = U(:,2);
 
end
 
 