%%do simulation first 
rng(1776)
axel_len = .62;
%dt = .02;
%time_to_solve = 0:dt:10;
delta_time = .02;
total_time = 10;%480;
time_to_solve = [0:delta_time:total_time];
left_cmd = zeros(1,length(time_to_solve));
right_cmd = zeros(1,length(time_to_solve));

[left_cmd , right_cmd, lj, rj] = genWheelTraj(time_to_solve);
%send commands through lead compensator
[A, B, C, D] = tf2ss([1,4],[1,20]); %move pole at -4 to -20
c2d(ss(A,B,C,D), .02)
state = [0;0];
output_record = zeros(2,length(time_to_solve));
for index = 1:length(time_to_solve)
    state = .6703 * state + .01648 * [left_cmd(index);right_cmd(index)];
    output_record(:,index) = 5*(-16*state + 1*[left_cmd(index);right_cmd(index)]);
end
%use lead compensated commands on system
left_wheel_cmd_vels  = [time_to_solve' , output_record(1,:)'];%left_cmd']; %packaged for simulink
right_wheel_cmd_vels = [time_to_solve' , output_record(2,:)'];%right_cmd'];


GGG = simset('Solver', 'ode4', 'FixedStep', delta_time)
sim('robotdynamic_simulink.slx', [0,time_to_solve(end)], GGG)

%% Take measurements (add noise)
IMUx_variance = .01^2;
IMUy_variance = .01^2;
IMUomega_variance = .01^2;

POSx_variance = .05^2;
POSy_variance = .05^2;
POStheta_variance = .05^2;

Ul_read_variance = .02^2;
Ur_read_variance = .02^2;


time_len = size(measurements.Data);
%give estimator input commands as planned
Ul_cmd_rec = left_cmd';%measurements.Data(:,1); %know input exactly
Ur_cmd_rec = right_cmd';%measurements.Data(:,2);

d2x_rec = measurements.Data(:,3) + sqrt(IMUx_variance) * randn(time_len(1), 1);
d2y_rec = measurements.Data(:,4) + sqrt(IMUy_variance) * randn(time_len(1), 1);
omega_rec = measurements.Data(:,5) + sqrt(IMUomega_variance) *randn(time_len(1), 1);

x_rec = measurements.Data(:,6) + sqrt(POSx_variance) *  randn(time_len(1), 1);
y_rec = measurements.Data(:,7) + sqrt(POSy_variance) *  randn(time_len(1), 1);
theta_rec = measurements.Data(:,8)   + sqrt(POStheta_variance) * randn(time_len(1), 1);
Ul_read_rec = measurements.Data(:,9)  + sqrt(Ul_read_variance) * randn(time_len(1), 1);
Ur_read_rec = measurements.Data(:,10) + sqrt(Ur_read_variance) * randn(time_len(1),1);

%% Set up estimator
syms theta omega Ul Ur dx dy d2x d2y d3x d3y dUl dUr d2Ul d2Ur
dt = .02; AxelLen = .62;

dx = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (cos(theta)*sin((Ur - Ul) / AxelLen)... %take dt off and then do expm?
                                 + sin(theta)*cos((Ur - Ul) / AxelLen) - sin(theta));
dy = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (sin(theta)*sin((Ur - Ul) / AxelLen)...
                                 - cos(theta)*cos((Ur - Ul) / AxelLen) + cos(theta));
                             
%thankfully:
%omega = (Ur - Ul) / AxelLen
%alpha;% = (dUr - dUl)/AxelLen;
%beta;% = (d2Ur - d2Ul)/AxelLen;

%d2x = diff(dx, Ul)*dUl + diff(dx, Ur)*dUr...
%    + diff(dx, theta)*(Ur - Ul) / AxelLen;
%d2y = diff(dy, Ul)*dUl + diff(dy, Ur)*dUr...
%    + diff(dy, theta)*(Ur - Ul) / AxelLen;
firstordermodel_dx = matlabFunction(dx);
firstordermodel_dy = matlabFunction(dy);

d2x = (firstordermodel_dx(Ul +dUl*dt, Ur +dUr*dt, theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_dx(Ul - dUl*dt, Ur -dUr*dt, theta -dt*(Ur-Ul)/AxelLen))/(2*dt);
d2y = (firstordermodel_dy(Ul +dUl*dt, Ur +dUr*dt, theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_dy(Ul - dUl*dt, Ur -dUr*dt, theta -dt*(Ur-Ul)/AxelLen))/(2*dt);

firstordermodel_d2x = matlabFunction(d2x);
firstordermodel_d2y = matlabFunction(d2y);

d3x = (firstordermodel_d2x(Ul + dUl*dt, Ur + dUr*dt,...
                           dUl + d2Ul*dt, dUr + d2Ur*dt,...
                           theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_d2x(Ul - dUl*dt, Ur - dUr*dt,...
                           dUl - d2Ul*dt, dUr - d2Ur*dt,...
                           theta - dt*(Ur-Ul)/AxelLen))/(2*dt);
d3y = (firstordermodel_d2y(Ul + dUl*dt, Ur + dUr*dt,...
                           dUl + d2Ul*dt, dUr + d2Ur*dt,...
                           theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_d2y(Ul - dUl*dt, Ur - dUr*dt,...
                           dUl - d2Ul*dt, dUr - d2Ur*dt,...
                           theta - dt*(Ur-Ul)/AxelLen))/(2*dt);
% 
% d3x = diff(d2x, Ul)*dUl      + diff(d2x, Ur)*dUr...
%     + diff(d2x, dUl)*d2Ul    + diff(d2x, dUr)*d2Ur...
%     + diff(d2x, theta)*(Ur - Ul) / AxelLen;
%     %+ diff(d2x, omega)*(dUr - dUl)/AxelLen;...
%     %+ diff(d2x, alpha)*(d2Ul - d2Ur)/AxelLen;
% 
% d3y = diff(d2y, Ul)*dUl      + diff(d2y, Ur)*dUr...
%     + diff(d2y, dUl)*d2Ul    + diff(d2y, dUr)*d2Ur...
%     + diff(d2y, theta)*(Ur - Ul) / AxelLen;
%     %+ diff(d2y, omega)*alpha...
%     %+ diff(d2y, alpha)*(d2Ul - d2Ur)/AxelLen;



%now take partial wrt each state (given the current input)
%state = [x,y,theta, dx, dy, d2x, d2y, Ul, Ur, dUl, dUr]
phi = [0, 0, diff(dx,theta), 0, 0, 0, 0, diff(dx, Ul),diff(dx,Ur), 0, 0; %dx = dx
       0, 0, diff(dy,theta), 0, 0, 0, 0, diff(dy, Ul),diff(dy,Ur), 0, 0; %dy = dy
       0, 0, 0, 0, 0, 0, 0, -1/AxelLen, 1/AxelLen, 0, 0; %dth = om
       0, 0, diff(d2x,theta), 0, 0, 0, 0, ...
       diff(d2x,Ul), diff(d2x,Ur), diff(d2x,dUl), diff(d2x,dUr); %ddx = d2x
       0, 0, diff(d2y,theta), 0, 0, 0, 0, ...
       diff(d2y,Ul), diff(d2y,Ur), diff(d2y,dUl), diff(d2y,dUr); %ddy = d2y
       0, 0, 0, 0, 0, ...
       0, 0, diff(d3x, Ul), diff(d3x, Ur),...
             diff(d3x, dUl),diff(d3x, dUr); %d2x
       0, 0, 0, 0, 0, ...
       0, 0, diff(d3y, Ul), diff(d3y, Ur),...
             diff(d3y, dUl),diff(d3y, dUr); %d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; %Ul %make it slow down a bit
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1; %Ur %on its own
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; %dUl
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];%dUr
 gamma = [0, 0; %x
          0, 0; %y
          0, 0; %theta
          0, 0; %dx
          0, 0; %dy
          diff(d3x, d2Ul), diff(d3x, d2Ur); %d2x
          diff(d3y, d2Ul), diff(d3y, d2Ur); %d2y
          0, 0; %Ul
          0, 0; %Ur
          1, 0; %dUl
          0, 1]; %dUr

 %now can turn into matlab function and use
 %need care for Ul = Ur
 phi_state = matlabFunction(phi)
  %reads Ul, Ur, d2Ul, d2Ur, dUl, dUr, theta as state
 gamma_state = matlabFunction(gamma)
  %reads Ul, Ur, oemga, theta
 %make sure Ul != Ur
 %phi_k is a function of state
 %Extended Kalman filter EQ's
 state_cov = diag([1, 1, 1,...
                   .1, .1,... 
                   .1, .1, .1, .1,...
                   .1, .1].^2);
               
delta_time = .02;
total_time = 10;%480;
time_to_solve = [0:delta_time:total_time];
               
 x = [0; 0; 0;...   x,  y, theta
      0; 0;...  dx, dy,
      0; 0; ...   d2x, d2y
      0; 0; ...    Ul,  Ur
      0; 0];...   dUl, dUr
 %left_u = @(t) ((t <= 4.0).*(2*(t <= .5) - 1).*2.8.*sin(1*2*pi*t))'
 %left_u = @(t) ((t<=6).*((t<=2).*.8.*sin(2*pi*(t/2)) -...
 %                (t>=4).*.8.*sin(2*pi*(t/2))))';
 %U = [1.8*left_u(time_to_solve), 2*left_u(time_to_solve)];
 U = [lj, rj];
 num_runs = length(time_to_solve);
 state_rec = zeros(length(x),num_runs); 
 
 %input noise
 Q = diag([.002, .002, .0002, ... %x, y, theta
           .005,  .005, .005, .005,... %dx, dy, d2x, d2y
           .002, .002, .002, .002].^2); %Ul, Ur, dUl, dUr
 %measurement noise
 R = diag([.05, .05, .05,... %x, y, theta
           .01, .01, .02, .02, .01].^2); %d2x, d2y, Ul, Ur, omega
 %measurement mat
 C = [eye(3), zeros(3, 8); %x, y, theta
      zeros(4, 5), eye(4, 6); % d2x, d2y, Ul, Ur
      zeros(1, 7), -1/AxelLen, 1/AxelLen, 0, 0]; %omega
 for index = 1:num_runs
     [A_k, B_k] = robust_phi_state(x, U(index,:), phi_state, gamma_state);
     p = ss(A_k, B_k, eye(11), zeros(11,2));
     pd = c2d(p, delta_time); %Get DT linearization
     x = pd.A*x + pd.B*U(index,:)'; %Propagate estimate
     x(3) = angleDiff(x(3),0);
     state_cov = pd.A*state_cov*pd.A' + Q; %Propagate covariance
     K = state_cov*C' / (C*state_cov*C' + R); %Calculate gains
     
     meas = [x_rec(index);     y_rec(index); theta_rec(index);...
            d2x_rec(index); d2y_rec(index); ...
            Ul_read_rec(index); Ur_read_rec(index); ...
            omega_rec(index)];
        
     x = x + K*(meas - C*x); %Update estimate with measurement
     state_cov = (eye(11) - K*C)*state_cov; %Update covariance wrt gains
     if abs(x(3) - meas(3)) > 1.8*pi
         x(3) = meas(3)
     end
     state_rec(:,index) = x;
 end
 
 %%Plot the results
 figure();
 %subplot(2, 1, 1);
 hold on
 plot(measurements.Time, x_rec, 'b*', 'MarkerSize', 1)
 plot(measurements.Time, y_rec, 'r*', 'MarkerSize', 1)
 plot(measurements.Time, theta_rec, 'c*', 'MarkerSize', 1)
 %plot(measurements.Time, omega_rec, 'g*', 'MarkerSize', 1)
 plot(measurements.Time, d2x_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.2 .4 .6])
 plot(measurements.Time, d2y_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.6 .2 .4])
 plot( time_to_solve,state_rec(1, :),'b--');
 plot(time_to_solve, state_rec(2, :), 'r--');...,...
 plot(time_to_solve, state_rec(3, :), 'c--');
 plot(time_to_solve, state_rec(6, :), '--');...,...
 plot(time_to_solve, state_rec(7, :), '--');
 plot(time_to_solve, state_rec(4, :), '--');...,...
 plot(time_to_solve, state_rec(5, :), '--');
       %time_to_solve,x_rec(4, :), '--', time_to_solve, x_rec(5, :), '--', ...
       %time_to_solve,x_rec(6, :), '--', time_to_solve, x_rec(7, :), '--');
 plot(measurements.Time, delta_accum.Data(:, 1), 'b-')
 plot(measurements.Time, delta_accum.Data(:, 2), 'r-')
 plot(measurements.Time, delta_accum.Data(:, 3), 'c-')

  legend('x_{mea}', 'y_{mea}','theta_{mea}','d2x_{mea}', 'd2y_{mea}',...
       'x_{est}', 'y_{est}', 'theta_{est}','d2x_{est}', 'd2y_{est}',...
       'dx_{est}', 'dy_{est}',...
       'x_{tru}', 'y_{tru}', 'theta_{tru}');
 %legend('x', 'y',  'dx', 'dy', 'd2x', 'd2y');
 %subplot(2, 1, 2);
 figure();
hold on;
plot(measurements.Time, Ul_read_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, Ur_read_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.6 .2 .4])
plot(time_to_solve, state_rec(8, :), '--', 'MarkerFaceColor',[.2 .4 .6])
plot(time_to_solve, state_rec(9, :), '--', 'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, delta_accum.Data(:, 4), '-', 'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, delta_accum.Data(:, 5), '-', 'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, left_cmd, 'd', 'MarkerFaceColor',[.9 .4 .6])
plot(measurements.Time, right_cmd, 'd', 'MarkerFaceColor',[.9 .6 .4])
plot(time_to_solve, state_rec(10, :), '--', 'MarkerFaceColor',[.8 .8 .4])
plot(time_to_solve, state_rec(11, :), '--', 'MarkerFaceColor',[.4 .8 .8])
legend('Ul_{mea}', 'Ur_{mea}', 'Ul_{est}', 'Ur_{est}', 'Ul_{tru}','Ur_{tru}',...
       'Ul_{cmd}', 'Ur_{cmd}', 'dUl_{est}', 'dUr_{est}');
title('Wheel Velocities');

 
 function [phi, gamma] = robust_phi_state(x, u, phi_func, gamma_func)
 vel_adj = x(8);  
 if (abs(x(8) - x(9)) < .00001) 
       vel_adj = x(8) + .00002;
 end
       phi = phi_func(vel_adj, x(9), u(1), u(2), x(10), x(11),x(3));
       gamma = gamma_func(vel_adj, x(9), u(1), u(2), x(10), x(11),x(3));
       
% phi = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; %dx = dx
%        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0; %dy = dy
%        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0; %dth = om
%        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; %ddx = d2x
%        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; %ddy = d2y
%        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 1; %1/axel len really
%        0, 0, 0, 0, 0, 0, ...
%        0, 0, 0, 0, 0,...
%        0; %d2x
%        0, 0, 0, 0, 0, 0, ...
%        0, 0, 0,0,0,...
%        0; %d2y
%        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
%        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
%        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
%  gamma = [0, 0; %x
%           0, 0; %y
%           0, 0; %theta
%           0, 0; %dx
%           0, 0; %dy
%           0, 0; %omega
%           .5*cos(x(3)), .5*cos(x(3)); %d2x, is this correct?
%           .5*sin(x(3)), .5*sin(x(3)); %d2y
%           0, 0; %Ul
%           0, 0; %Ur
%           1, 0; %dUl
%           0, 1]; %dUr   
%    end
 end
   
       
       
       
       