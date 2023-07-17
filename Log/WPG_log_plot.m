clear;
close all;

load WPG_log_FootTrajectory.dat;
load WPG_log_WalkingPattern.dat;

[a, b] = size(WPG_log_FootTrajectory);

t = 1:a;
y = 0.01:0.05;
z = 0.01:0.2;

i=0;

for n=t
  i = i + 1;
  xc(i) = WPG_log_FootTrajectory(i, 1);
  yc(i) = WPG_log_FootTrajectory(i, 2);
  x1(i) = WPG_log_FootTrajectory(i, 3);
  y1(i) = WPG_log_FootTrajectory(i, 4);
  z1(i) = WPG_log_FootTrajectory(i, 5);
  x2(i) = WPG_log_FootTrajectory(i, 6);
  y2(i) = WPG_log_FootTrajectory(i, 7);
  z2(i) = WPG_log_FootTrajectory(i, 8);
end

f1 = figure;
f2 = figure;
figure(f1);
plot3(t, xc, z1, t, y1, z1, t, y2, z2)
legend("-", "Foot Pos of Support Leg relative to CoG Pos (t, y, z)", "Foot Pos of Swing Leg relative to CoG Pos (t, y, z)")
grid on

figure(f2);
plot(t, xc, t, y1, t, y2)
legend("-", "Foot Pos of Support Leg relative to CoG Pos (t, y)", "Foot Pos of Swing Leg relative to CoG Pos (t, y)")
grid on

f3 = figure;
figure(3);
plot(t, WPG_log_WalkingPattern)
legend(["CoG Position X world", "CoG Position Y world", "CoG Position X local", "CoG Position Y local", "CoG Velocity X", "CoG Velocity Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
xlabel("time")
grid on

% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html