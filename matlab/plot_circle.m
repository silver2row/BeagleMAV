function plot_circle(x_mid, y_mid, radius, colour)

x = x_mid+radius*cos(linspace(0, 2*pi,100));
y = y_mid+radius*sin(linspace(0, 2*pi,100));
plot(x, y, colour)

end