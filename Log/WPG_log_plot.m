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
% legend(["CoG Position X world", "CoG Position Y world", "CoG Velocity X", "CoG Velocity Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
legend(["CoG Position X world", "CoG Position Y world", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
xlabel("time")
grid on

load WPG_log_FootTrajectory_FK.dat;

[a4, b4] = size(WPG_log_FootTrajectory_FK);

t4 = 1:a4;

i = 0;
for n=t
  i = i+1;
  xr(i) = WPG_log_FootTrajectory_FK(i, 1);
  yr(i) = WPG_log_FootTrajectory_FK(i, 2);
  zr(i) = WPG_log_FootTrajectory_FK(i, 3);
  xl(i) = WPG_log_FootTrajectory_FK(i, 4);
  yl(i) = WPG_log_FootTrajectory_FK(i, 5);
  zl(i) = WPG_log_FootTrajectory_FK(i, 6);
end

f4 = figure;
figure(f4);
plot3(t4, yr, zr, t4, yl, zl)
legend("right foot", "left foot")
xlabel("time")
ylabel("y position")
zlabel("z position")
grid on

f5 = figure;
figure(f5);
plot(t4,xr, t4, xl)
legend("right", "left")
xlabel("time")
ylabel("x position")
grid on

f6 = figure;
figure(f6);
plot3(xr, yr, zr, xl, yl, zl)
legend("right foot", "left foot")
xlabel("x position")
ylabel("y position")
grid on
zlabel("z position")

load WPG_log_SwingTrajectory.dat;

f7 = figure;
figure(f7);
plot(t4, WPG_log_SwingTrajectory)
xlabel("time")
ylabel("swing trajectory")
grid on

% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html