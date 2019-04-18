%
% [x,y, theta, dx, dy, omega, d2x, d2y, Ul, Ur, dUl, dUr]

%domega = dUr - dUl

%dUl = input L
%dUr = input R

%d3x = 
%d3y = 

syms theta omega Ul Ur dx dy d2x d2y d3x d3y dUl dUr d2Ul d2Ur
dt = .002; AxelLen = .62;

dx = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (cos(theta)*sin((Ur - Ul) / AxelLen)... %take dt off and then do expm?
                                 + sin(theta)*cos((Ur - Ul) / AxelLen) - sin(theta));
dy = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (sin(theta)*sin((Ur - Ul) / AxelLen)...
                                 - cos(theta)*cos((Ur - Ul) / AxelLen) + cos(theta));
                             
%thankfully:
%omega = (Ur - Ul) / AxelLen
%alpha;% = (dUr - dUl)/AxelLen;
%beta;% = (d2Ur - d2Ul)/AxelLen;

d2x = diff(dx, Ul)*dUl + diff(dx, Ur)*dUr...
    + diff(dx, theta)*(Ur - Ul) / AxelLen;
d2y = diff(dy, Ul)*dUl + diff(dy, Ur)*dUr...
    + diff(dy, theta)*(Ur - Ul) / AxelLen;

d3x = diff(d2x, Ul)*dUl      + diff(d2x, Ur)*dUr...
    + diff(d2x, dUl)*d2Ul    + diff(d2x, dUr)*d2Ur...
    + diff(d2x, theta)*(Ur - Ul) / AxelLen;
    %+ diff(d2x, omega)*(dUr - dUl)/AxelLen;...
    %+ diff(d2x, alpha)*(d2Ul - d2Ur)/AxelLen;

d3y = diff(d2y, Ul)*dUl      + diff(d2y, Ur)*dUr...
    + diff(d2y, dUl)*d2Ul    + diff(d2y, dUr)*d2Ur...
    + diff(d2y, theta)*(Ur - Ul) / AxelLen;
    %+ diff(d2y, omega)*alpha...
    %+ diff(d2y, alpha)*(d2Ul - d2Ur)/AxelLen;



%now take partial wrt each state (given the current input)
%state = [x,y,theta, dx, dy, d2x, d2y, Ul, Ur, dUl, dUr]
phi = [0, 0, 0, 0, 0, 0, 0, cos(theta)/2, cos(theta)/2, 0, 0; %dx = dx
       0, 0, 0, 0, 0, 0, 0, sin(theta)/2, sin(theta)/2, 0, 0; %dy = dy
       0, 0, 0, 0, 0, 0, 0, -1/AxelLen, 1/AxelLen, 0, 0; %dth = om
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; %ddx = d2x
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; %ddy = d2y
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
total_time = 5.3;%480;
time_to_solve = [0:delta_time:total_time];
               
 x = [0; 0; 0;...   x,  y, theta
      0; 0;...  dx, dy,
      0; 0; ...   d2x, d2y
      0; 0; ...    Ul,  Ur
      0; 0];...   dUl, dUr
 %left_u = @(t) ((t <= 4.0).*(2*(t <= .5) - 1).*2.8.*sin(1*2*pi*t))'
 left_u = @(t) ((t<=4).*((t<=2).*2.8.*sin(2*pi*(t/2)) -...
                 (t>2).*2.8.*sin(2*pi*(t/2))))';
 U = [left_u(time_to_solve), 2*left_u(time_to_solve)];

 num_runs = length(time_to_solve);
 x_rec = zeros(length(x),num_runs); 
               
 for index = 1:num_runs
     [phi_k, gamma_k] = robust_phi_state(x, U(index,:), phi_state, gamma_state);
     %dt_phi = expm(phi_k * delta_time);
     %dt_gamma = phi_k \ (dt_phi - eye(12))*gamma_k;
     p = ss(phi_k, gamma_k, eye(11), zeros(11,2));
     pd = c2d(p, delta_time);
     x = pd.A*x + pd.B*U(index,:)';
     x_rec(:,index) = x;
 end
 
 figure();
 subplot(2, 1, 1);
 plot( time_to_solve,x_rec(1, :), time_to_solve, x_rec(2, :),...
       time_to_solve,x_rec(4, :), time_to_solve, x_rec(5, :), ...
       time_to_solve,x_rec(6, :), time_to_solve, x_rec(7, :));
 legend('x', 'y',  'dx', 'dy', 'd2x', 'd2y');
 subplot(2, 1, 2);
 plot(time_to_solve, x_rec(3, :),...
      time_to_solve, x_rec(8, :), time_to_solve, x_rec(9, :), ...
      time_to_solve, x_rec(10,:), time_to_solve, x_rec(11,:));
 legend('theta','Ul', 'Ur', 'dUl', 'dUr');
 function [phi, gamma] = robust_phi_state(x, u, phi_func, gamma_func)
 vel_adj = x(8);  
 if (abs(x(8) - x(9)) < .01) 
       vel_adj = x(8) + .02;
 end
       phi = phi_func(vel_adj, x(9), u(1), u(2), x(10), x(11),x(3));
       gamma = gamma_func(vel_adj, x(9), x(3));
       
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
   
       
       
       
       