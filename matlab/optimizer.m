%% James Strawson 2015
%% This uses the fminsearch matlab function to find the rotor layout
%% that minimizes the multirotor performance objective function
%% define in objective.m

clear all
close all
clc

% starting conditions
p = [-0.1  % p1x
      -0.2  % p1y
     -0.1  % p2x
      0.2  % p2y
     -0.2]; % p3x
    
%% search for optimum
[p,fval, exitflag] = fminsearch(@objective, p);

%% print results
%  fval
%  p
%  exitflag
 
 %% Components of thrust vector for right 3 rotors
% x components are mirrored to the left props
% sum of y components must be zero
p1x = p(1)
p1y = p(2)
p2x = p(3)
p2y = p(4)
p3x = p(5)
p3y =  -(p1y+p2y) 



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


 %% plot the result
 n=6;
 r = 175; %radius of hexagon in mm
angle   = [ 60  0 300 240  180  120]'; 
radius  = [ r   r    r    r    r    r ]';
rotation  = [1   -1   1    -1   1    -1 ]';

%% rotor locations in cartesian coordinates
x = (cos(angle*2*pi/360).*radius);
y = (sin(angle*2*pi/360).*radius);

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