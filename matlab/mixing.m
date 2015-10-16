%% messing around mixing propeller inputs for multirotors
%% James Strawson 2015

clear all
close all
clc

%% layouts 
% each is given firt in radial coordinates in degrees. Angle is CCW from
% the X axis. Looking top-down on the multirotor, Y axis points forward,
% X axis point starboard
% the z vector is the spin direction of rotors. 1 is CCW, -1 is CW
% radial coordinates are converted to cartesian after
% vector should start with the rotor on the positive X axis or just above
% then work around CCW
% Uncomment a section to use it

% %% quad X
angle   = [  45   135  -135  -45];
radius  = [   1    1     1     1];
z       = [   1   -1     1    -1];
% 
% %% quad +
% angle   = [   0   90   180   270];
% radius  = [   1   1     1     1];
% z       = [   1  -1     1    -1];
% 
% % hex X
angle   = [ 0   60  120  180  240  300]; 
radius  = [ 1   1    1    1    1    1 ];
z       = [-1   1   -1    1   -1    1 ];
% 
% % hex triangle
% angle   = [ 90   90  210  210  330  330]; 
% radius  = [ 1   1    1    1    1    1 ];
% z       = [-1   1   -1    1   -1    1 ];
% 
% % %% hex triangle long
% angle   = [ 90   90  230  230  310  310]; 
% radius  = [ 1   1    1    1    1    1 ];
% z       = [-1   1   -1    1   -1    1 ];

%% hex +
% angle   = [ 30  90  150  210  270  330]; 
% radius  = [ 1   1    1    1    1    1 ];
% z       = [ 1  -1    1   -1    1   -1 ];
% % 
% %% octo X
% angle   = [  22.5   67.5  112.5  157.5  202.5  247.5  292.5  337.5]; 
% radius  = [  1   1    1    1    1    1    1    1 ];
% z       = [  1  -1    1   -1    1   -1    1   -1 ];
% % 
% %% octo +
% angle   = [ 0  45  90  135  180  -135   -90   -45]; 
% radius  = [ 1   1    1    1    1    1    1     1 ];
% z       = [-1   1   -1    1   -1    1    -1    1 ];

% %% experimenting
% angle   = [  45   135  -135  -45];
% radius  = [   2    1     1     1];
% z       = [   1   -1     1    -1];


%% convert to cartesian coordinates and extract the # of rotors n
disp('rotor locations');
x = cos(angle*2*pi/360).*radius
y = sin(angle*2*pi/360).*radius
[n,m] = size(x');
fprintf('rotors: %d\n', n);

%% plot the rotors so we can sanity check the layout
fig1 = figure();
set(fig1, 'Units', 'normalized');
set(fig1, 'Position', [.2 .2 .7 .7]);
hold on
for i = 1:n
   if(z(i)>0)
       plot_circle(x(i), y(i), 0.5, 'r')
   else
       plot_circle(x(i), y(i), 0.5, 'b')
   end
end
axis square
title('red CCW, blue CW')

%% constraints given by these euqations
% R*x' = 1;   % net yaw something
% R*y' = 0;   % net pitch zero
% R*z' = 0;   % net yaw zero
% R*ones(1,n) = 0; % net thrust zero

% solve for roll mixing
% R: final mixing constants
% A: frame information
% c: conditions, see above
% R*A=c
A = [-x; y; z; ones(1,n)]';
c = [n 0 0 0];
R = c*pinv(A);
R = R/(abs(max(R)))

% solve for pitch mixing
A = [y; x; z; ones(1,n)]';
c = [n 0 0 0];
P = c*pinv(A);
P = P/(abs(max(P)))

% solve for yaw mixing
% positive control input results in CW (negative) moment on frame, 
% thus -n such that moment is positive once mixed
A = [z; x; z; ones(1,n)]';
c = [-n 0 0 0];
Y = c*pinv(A);
Y = Y/(abs(max(Y)))

% solve for throttle mixing
A = [ones(1,n); y; x; z]' ;
c = [n 0 0 0];
T = c*pinv(A);
T = T/(abs(max(T)))
