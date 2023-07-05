clear;

load WPG_log.dat;

[a, b] = size(WPG_log);

t = 1:a;
y = 0.01:0.05;
z = 0.01:0.2;

i=0;

for n=t
  i = i + 1;
  x1(i) = WPG_log(i, 1);
  y1(i) = WPG_log(i, 2);
  z1(i) = WPG_log(i, 3);
  x2(i) = WPG_log(i, 4);
  y2(i) = WPG_log(i, 5);
  z2(i) = WPG_log(i, 6);
end

% subplot(1, 2, 1)
plot3(t, y1, z1, t, y2, z2)
legend("CoG Pos", "CoG Pos Swing Trajectory")
grid on

% subplot(1, 2, 2)
% plot(t, y1, t, y2)
% legend("CoG Pos", "CoG Pos Swing Trajectory")
% plot3(WPG_log)

% legend(["CoG Position X", "CoG Position Y", "CoG Velocity X", "CoG Velocity Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
% legend({"CoG Position X", "CoG Position Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"}, "Location", "northwest")
% xlabel("time")

% grid on

% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html