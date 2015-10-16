function [ O ] = objective( p, C )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


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


%% Components of thrust vector for right 3 rotors
% x components are mirrored to the left props
% sum of y components must be zero
p1x = p(1);
p1y = p(2);
p2x = p(3);
p2y = p(4);
p3x = p(5);
p3y =  -(p1y+p2y);  % enforce sum of y components are 0
%p3y = p(6);

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
R_mix = R_mix/(max(abs(R_mix)));

%% solve for Pitch
c = [0 1 0 0 0 0];
P_mix = c*pinv(A);
P_mix = P_mix/(max(abs(P_mix)));

%% solve for Yaw mixing
c = [0 0 1 0 0 0];
Yaw_mix = c*pinv(A);
Yaw_mix = Yaw_mix/(max(abs(Yaw_mix)));

%% solve for Upwards Thrust
c = [0 0 0 1 0 0];
T_mix = c*pinv(A);
T_mix = T_mix/(max(abs(T_mix)));

%% solve for X Thrust
c = [0 0 0 0 1 0];
X_mix = c*pinv(A);
X_mix = X_mix/(max(abs(X_mix)));

%% solve for Y Thrust
c = [0 0 0 0 0 1];
Y_mix = c*pinv(A);
Y_mix = Y_mix/(max(abs(Y_mix)));



%% Cost Function

Net = [R_mix * (-x.*tv_z)
        P_mix * (y.*tv_z)
        Yaw_mix * M_yaw
        T_mix * tv_z
        X_mix * tv_x
        Y_mix * tv_y]

C = [0.5 0.5 1 5000 500 1500];




O =  - (C * Net);

% if(p(3)>0)
%     O = O + 10000;
% end

% if(p3y<0)
%     O = O + 10000;
% end


end

