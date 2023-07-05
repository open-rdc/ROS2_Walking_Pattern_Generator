clear;

load WPG_log.dat;

[a, b] = size(WPG_log);

t = 1:a;
y = 0.01:0.05;
z = 0.01:0.2;

plot3(WPG_log)

% legend(["CoG Position X", "CoG Position Y", "CoG Velocity X", "CoG Velocity Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
% legend({"CoG Position X", "CoG Position Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"}, "Location", "northwest")
% xlabel("time")
grid on

% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html