%
% [x,y, theta, dx, dy, omega, d2x, d2y, Ul, Ur, dUl, dUr]

%domega = dUr - dUl

%dUl = input L
%dUr = input R

%d3x = 
%d3y = 

syms theta omega Ul Ur dx dy d2x d2y d3x d3y dUl dUr d2Ul d2Ur
dt = .02; AxelLen = .62;

dx = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (cos(theta)*sin(omega)... %take dt off and then do expm?
                                 + sin(theta)*cos(omega) - sin(theta));
dy = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (sin(theta)*sin(omega)...
                                 - cos(theta)*cos(omega) + cos(theta));
                             
%thankfully:
%alpha;% = (dUr - dUl)/AxelLen;
%beta;% = (d2Ur - d2Ul)/AxelLen;

d2x = diff(dx, Ul)*dUl + diff(dx, Ur)*dUr...
    + diff(dx, theta)*omega + diff(dx, omega)*(dUr - dUl)/AxelLen;
d2y = diff(dy, Ul)*dUl + diff(dy, Ur)*dUr...
    + diff(dy, theta)*omega + diff(dy, omega)*(dUr - dUl)/AxelLen;

d3x = diff(d2x, Ul)*dUl      + diff(d2x, Ur)*dUr...
    + diff(d2x, theta)*omega + diff(d2x, omega)*(dUr - dUl)/AxelLen...
    + diff(d2x, dUl)*d2Ul    + diff(d2x, dUr)*d2Ur;
    %+ diff(d2x, omega)*(dUr - dUl)/AxelLen;...
    %+ diff(d2x, alpha)*(d2Ul - d2Ur)/AxelLen;

d3y = diff(d2y, Ul)*dUl      + diff(d2y, Ur)*dUr...
    + diff(d2y, theta)*omega + diff(d2y, omega)*(dUr - dUl)/AxelLen...
    + diff(d2y, dUl)*d2Ul    + diff(d2y, dUl)*d2Ul;
    %+ diff(d2y, omega)*alpha...
    %+ diff(d2y, alpha)*(d2Ul - d2Ur)/AxelLen;



%now take partial wrt each state (given the current input)
%state = [x,y,theta, dx, dy, omega, d2x, d2y, Ul, Ur, dUl, dUr]
phi = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; %dx = dx
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0; %dy = dy
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0; %dth = om
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; %ddx = d2x
       0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; %ddy = d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 1; %1/axel len really
       0, 0, diff(d3x, theta), 0, 0, diff(d3x,omega), ...
       0, 0, diff(d3x, Ul), diff(d3x, Ur), diff(d3x, dUl),...
       diff(d3x, dUr); %d2x
       0, 0, diff(d3y, theta), 0, 0, diff(d3y,omega), ...
       0, 0, diff(d3y, Ul), diff(d3y, Ur), diff(d3y, dUl),...
       diff(d3y, dUr); %d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; %Ul
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1; %Ur
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; %dUl
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];%dUr
 gamma = [0, 0; %x
          0, 0; %y
          0, 0; %theta
          0, 0; %dx
          0, 0; %dy
          0, 0; %omega
          diff(d3x, d2Ul), diff(d3x, d2Ur); %d2x
          diff(d3y, d2Ul), diff(d3y, d2Ur); %d2y
          0, 0; %Ul
          0, 0; %Ur
          1, 0; %dUl
          0, 1]; %dUr

 %now can turn into matlab function and use
 %need care for Ul = Ur
 phi_state = matlabFunction(phi)
  %reads Ul, Ur, d2Ul, d2Ur, dUl, dUr, omega, theta as state
 gamma_state = matlabFunction(gamma)
  %reads Ul, Ur, oemga, theta
 %make sure Ul != Ur
 %phi_k is a function of state
 %Extended Kalman filter EQ's
 state_cov = diag([1, 1, 1,...
                   .1, .1, .1,... 
                   .1, .1, .1, .1,...
                   .1, .1].^2);
               
delta_time = .02;
total_time = 10;%480;
time_to_solve = [0:delta_time:total_time];
               
 x = [0; 0; 0;...   x,  y, theta
      0; 0; 0;...  dx, dy, omega
      0; 0; ...   d2x, d2y
      0; 0; ...    Ul,  Ur
      0; 0];...   dUl, dUr
 U = [cos(time_to_solve)', sin(1.2*time_to_solve)'];

               
 for index = 1:3
     [phi_k, gamma_k] = robust_phi_state(x, U(index,:), phi_state, gamma_state)
     x = x + phi_k*x + gamma_k*U(index,:)';
 end
 
 function [phi, gamma] = robust_phi_state(x, u, phi_func, gamma_func)
   if (abs(u(1) - u(2)) > .001) && (abs(x(9) - x(10)) > .001) && (abs(x(11) - x(12)) > .001)
       phi = phi_func(x(9), x(10), u(1), u(2), x(11), x(12), x(6), x(3));
       gamma = gamma_func(x(9), x(10), x(6), x(3));
   else
phi = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; %dx = dx
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0; %dy = dy
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0; %dth = om
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; %ddx = d2x
       0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; %ddy = d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 1; %1/axel len really
       0, 0, 0, 0, 0, 0, ...
       0, 0, 0, 0, 0,...
       0; %d2x
       0, 0, 0, 0, 0, 0, ...
       0, 0, 0,0,0,...
       0; %d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
 gamma = [0, 0; %x
          0, 0; %y
          0, 0; %theta
          0, 0; %dx
          0, 0; %dy
          0, 0; %omega
          .5*cos(x(3)), .5*cos(x(3)); %d2x, is this correct?
          .5*sin(x(3)), .5*sin(x(3)); %d2y
          0, 0; %Ul
          0, 0; %Ur
          1, 0; %dUl
          0, 1]; %dUr   
   end
 end
   
       
       
       
       