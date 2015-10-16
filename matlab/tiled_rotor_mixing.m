%% messing around mixing propeller inputs for multirotors
%% with tilted rotors
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


%% hex X   top view
%  6  1       cw ccw      Y
% 5    2    ccw     cw    ^
%  4  3       cw ccw      |__> X
n = 6;

%% top-view position of rotors in cartesian
r = 175; %radius of hexagon in mm
angle   = [ 60  0 300 240  180  120]'; 
radius  = [ r   r    r    r    r    r ]';
rotation  = [1   -1   1    -1   1    -1 ]';

%% rotor locations in cartesian coordinates
x = (cos(angle*2*pi/360).*radius);
y = (sin(angle*2*pi/360).*radius);

%% this is the vector to optimize over the cost function
% components of unit thrust vetor for right 3 rotors
% p = [-0.1;  % p1x
%       0.0;  % p1y
%      -0.2;  % p2x
%      -0.2;  % p2y
%      -0.3]; % p3x

p = [ 0.1032
   -0.0218
   -0.1436
   -0.9739
    0.1198];
  
params = length(p)

%% Components of thrust vector for right 3 rotors
% x components are mirrored to the left props
% sum of y components must be zero
p1x = p(1);
p1y = p(2);
p2x = p(3);
p2y = p(4);
p3x = p(5);
p3y =  -(p1y+p2y);  % enforce sum of y components are 0


%% Normalized thrust vectors & yaw moment vector
% mirror right rotors into vectors for all 6
tv_x = [ p1x;
         p2x;
         p3x;
        -p3x;
        -p2x;
        -p1x];

tv_y = [ p1y;
         p2y;
         p3y;
         p3y;
         p2y;
         p1y];

% find component of thrust in z direction for unit thrust
tv_z = sqrt(1-(tv_x.^2 + tv_y.^2));

% find moment about yaw axis 
% moment is cross product of rotor position and thrust vector in x&y
M_yaw = x.*tv_y - y.*tv_x;
M_yaw = M_yaw + 20*rotation;



%% plot the rotors so we can sanity check the layout
fig1 = figure();
set(fig1, 'Units', 'normalized');
set(fig1, 'Position', [.2 .2 .7 .7]);
hold on
l=radius(1)/2;
for i = 1:n
    %plot thrust vector
    plot3([x(i);x(i)+tv_x(i)*l],[y(i);y(i)+tv_y(i)*l], [0 ; tv_z(i)*l])
    
    %plot from origin to base of vector
    plot3([0;x(i)],[0,y(i)],[0,0],'r')
end
%plot forward pointer
plot3([0;0],[0,radius(1)*1.5],[0,0],'r')
axis equal
title('thrust vectors')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Solve for Mixing Matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% solve for roll mixing
% R: rol mixing constants
% A: frame information
% c: conditions, see above
% R*A=c

% columns of A are rotor effects in 
% roll, pitch, yaw, thrust, X, Y
A = [-x.*tv_z, y.*tv_z, M_yaw, tv_z, tv_x, tv_y];

%% solve for Roll
c = [1 0 0 0 0 0];
R_mix = c*pinv(A);
R_mix = R_mix/(max(abs(R_mix)))

%% solve for Pitch
c = [0 1 0 0 0 0];
P_mix = c*pinv(A);
P_mix = P_mix/(max(abs(P_mix)))

%% solve for Yaw mixing
c = [0 0 1 0 0 0];
Yaw_mix = c*pinv(A);
Yaw_mix = Yaw_mix/(max(abs(Yaw_mix)))

%% solve for Upwards Thrust
c = [0 0 0 1 0 0];
T_mix = c*pinv(A);
T_mix = T_mix/(max(abs(T_mix)))

%% solve for X Thrust
c = [0 0 0 0 1 0];
X_mix = c*pinv(A);
X_mix = X_mix/(max(abs(X_mix)))

%% solve for Y Thrust
c = [0 0 0 0 0 1];
Y_mix = c*pinv(A);
Y_mix = Y_mix/(max(abs(Y_mix)))



%% Cost Function

Net = [R_mix * (-x.*tv_z)
        P_mix * (y.*tv_z)
        Yaw_mix * M_yaw
        T_mix * tv_z
        X_mix * tv_x
        Y_mix * tv_y];

C = [1 1 1 200 50 50];

O = 999999 - ( C * Net)


