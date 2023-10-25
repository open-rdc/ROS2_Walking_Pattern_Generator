t = 0.0:0.01:0.8;

x = pi/0.8;

plot(t, 0.05*sin(x*t))

xlabel("time[s]")
ylabel("position z[m]")

grid on