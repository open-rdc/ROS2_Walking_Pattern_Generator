clear;

load WPG_log.dat;

[a, b] = size(WPG_log);

t = 1:a;
y = 0.01:0.05;
z = 0.01:0.2;

i=0;

for n=t
  i = i + 1;
  xc(i) = WPG_log(i, 1);
  yc(i) = WPG_log(i, 2);
  x1(i) = WPG_log(i, 3);
  y1(i) = WPG_log(i, 4);
  z1(i) = WPG_log(i, 5);
  x2(i) = WPG_log(i, 6);
  y2(i) = WPG_log(i, 7);
  z2(i) = WPG_log(i, 8);
end

f1 = figure;
f2 = figure;
figure(f1);
plot3(t, xc, z1, t, y1, z1, t, y2, z2)
legend("CoG Pos", "Foot Pos of Support Leg relative to CoG Pos", "Foot Pos of Swing Leg relative to CoG Pos")
grid on

figure(f2);
plot(t, xc, t, y1, t, y2)
legend("CoG Pos", "Foot Pos of Support Leg relative to CoG Pos", "Foot Pos of Swing Leg relative to CoG Pos")
grid on

% plot(t, WPG_log)
% legend(["CoG Position X", "CoG Position Y", "CoG Position X local", "CoG Position Y local", "CoG Velocity X", "CoG Velocity Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
% % legend({"CoG Position X", "CoG Position Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"}, "Location", "northwest")
% xlabel("time")

% grid on

% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html