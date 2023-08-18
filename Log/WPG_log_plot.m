clear;
close all;

%{

load WPG_log_FootTrajectory.dat;

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

f1 =  "WPG_log_FootTrajectory";
figure('name', f1);
plot3(t, y1, z1, t, y2, z2)
title("支持脚・遊脚の足先位置の時間変化")
legend("Foot Pos of Support Leg relative to CoG Pos", "Foot Pos of Swing Leg relative to CoG Pos")
xlabel("time")
ylabel("y position [m]")
zlabel("z position [m]")
grid on

f2 =  "WPG_log_FootTrajectory";
figure('name', f2);
plot(t, xc, t, y1, t, y2)
legend("-", "Foot Pos of Support Leg relative to CoG Pos (t, y)", "Foot Pos of Swing Leg relative to CoG Pos (t, y)")
grid on

load WPG_log_WalkingPattern.dat;

f3 = "WPG_log_WalkingPattern";
figure('name', f3);
plot(t, WPG_log_WalkingPattern)
% legend(["CoG Position X world", "CoG Position Y world", "CoG Velocity X", "CoG Velocity Y", "p x fix", "p y fix", "LandingPosition X", "LandingPosition Y"], "Location", "northwest")
legend(["CoG Position X world [m]", "CoG Position Y world [m]", "p x fix [m]", "p y fix [m]", "LandingPosition X [m]", "LandingPosition Y [m]"], "Location", "northwest")
xlabel("time")
ylabel("position [m]")
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

f4 = "WPG_log_FootTrajectory_FK";
figure('name', f4);
plot3(t4, yr, zr, t4, yl, zl)
title("左脚・右脚の足先位置の時間変化")
legend("Foot Pos of Right Leg relative to CoG Pos", "Foot Pos of Left Leg relative to CoG Pos")
xlabel("time")
ylabel("y position [m]")
zlabel("z position [m]")
grid on

figure("name", "hikaku")
plot3(t, y1, z1, t, y2, z2, t4, yr, zr, t4, yl, zl)
title("IK前（figure1）とIK->FK後（figure4）の比較")
legend("Foot Pos of Support Leg relative to CoG Pos", "Foot Pos of Swing Leg relative to CoG Pos", "Foot Pos of Right Leg relative to CoG Pos", "Foot Pos of Left Leg relative to CoG Pos")
xlabel("time")
ylabel("y position [m]")
zlabel("z position [m]")
grid on

f5 = "WPG_log_FootTrajectory_FK";
figure('name', f5);
plot(t4,xr, t4, xl)
legend("Foot Pos of Right Leg relative to CoG Pos", "Foot Pos of Left Leg relative to CoG Pos")
xlabel("time")
ylabel("x position [m]")
grid on

f6 = "WPG_log_FootTrajectory_FK";
figure('name', f6);
plot3(xr, yr, zr, xl, yl, zl)
legend("Foot Pos of Right Leg relative to CoG Pos", "Foot Pos of Left Leg relative to CoG Pos")
xlabel("x position [m]")
ylabel("y position [m]")
grid on
zlabel("z position")

load WPG_log_SwingTrajectory.dat;

f7 = "WPG_log_SwingTrajectory";
figure('name', f7);
plot(t4, WPG_log_SwingTrajectory)
xlabel("time")
ylabel("z position [m]")
legend("swing trajectory", "old swing trajectory")
grid on

load WPG_log_SwingTrajectory_Vel.dat;

[a8, b8] = size(WPG_log_SwingTrajectory_Vel);
t8 = 1:a8;
i = 0;
for n=t8
  i = i + 1;
  dx_su(i) = WPG_log_SwingTrajectory_Vel(i, 1);
  dy_su(i) = WPG_log_SwingTrajectory_Vel(i, 2);
  dz_su(i) = WPG_log_SwingTrajectory_Vel(i, 3);
  dx_sw(i) = WPG_log_SwingTrajectory_Vel(i, 7);
  dy_sw(i) = WPG_log_SwingTrajectory_Vel(i, 8);
  dz_sw(i) = WPG_log_SwingTrajectory_Vel(i, 9);
end

f8 = "WPG_log_SwingTrajectory_Vel";
figure("name", f8);
plot(t8, dz_sw)
legend("dz: swing leg")
xlabel("time")
ylabel("z trans vel [m/s]")
grid on

%}

load WRH_log_Feedback.dat;

[a9, b9] = size(WRH_log_Feedback);
t9 = 1:a9;
i = 0;
for n=t9
  i = i + 1;
  acc_x(i) = WRH_log_Feedback(i, 2);
  acc_y(i) = WRH_log_Feedback(i, 3);
  acc_z(i) = WRH_log_Feedback(i, 4);
  gyr_x(i) = WRH_log_Feedback(i, 5);
  gyr_y(i) = WRH_log_Feedback(i, 6);
  gyr_z(i) = WRH_log_Feedback(i, 7);
end

f9 = "WRH_log_Feedback";
figure("name", f9);

subplot(2, 3, 1)
plot(t9, acc_x)
title("Accelerometer axis-X")
xlabel("step")
grid on

subplot(2, 3, 2)
plot(t9, acc_y)
title("Accelerometer axis-Y")
xlabel("step")
grid on

subplot(2, 3, 3)
plot(t9, acc_z)
title("Accelerometer axis-Z")
xlabel("step")
grid on

subplot(2, 3, 4)
plot(t9, gyr_x)
title("Gyro axis-X")
xlabel("step")
grid on

subplot(2, 3, 5)
plot(t9, gyr_y)
title("Gyro axis-Y")
xlabel("step")
grid on

subplot(2, 3, 6)
plot(t9, gyr_z)
title("Gyro axis-Z")
xlabel("step")
grid on


% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html