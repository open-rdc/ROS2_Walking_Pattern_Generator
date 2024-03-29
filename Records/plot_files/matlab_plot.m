clear all;
close all;

n = newline;

load("../2023-12-26_16-47-32/feedback0.dat");
load("../2023-12-26_16-47-32/foot_step0.dat");
load("../2023-12-26_16-47-32/joint_states0.dat");
load("../2023-12-26_16-47-32/walking_pattern0.dat");
load("../2023-12-26_16-47-32/walking_stabilization0.dat");

[nf, pf] = size(feedback0);
[nw, pw] = size(walking_pattern0);
[ns, ps] = size(foot_step0);
[nj, pj] = size(joint_states0);

t1feedback = 1:nf;
t1walking = 1:nw;
t1footstep = 1:ns;
for t = 1:ns
  foot_step0(t,3) = foot_step0(t,3) - 0.037;
end
t1jointstates = 1:nj;

figure;
subplot(2,3,1)
plot(t1feedback(1:700), feedback0(1:700,2));
title("Accelerometer X"+n+"(mapping: 0~1024. -39.24~39.24[m/s^2])")
xlabel("step")
ylabel("acceleration (0~1024)")
grid on
%https://cyberbotics.com/doc/reference/accelerometer
subplot(2,3,2)
plot(t1feedback(1:700), feedback0(1:700,3));
title("Accelerometer Y"+n+"(mapping: 0~1024. -39.24~39.24[m/s^2])")
xlabel("step")
ylabel("acceleration (0~1024)")
grid on

subplot(2,3,3)
plot(t1feedback(1:700), feedback0(1:700,4));
title("Accelerometer Z"+n+"(mapping: 0~1024. -39.24~39.24[m/s^2])")
xlabel("step")
ylabel("acceleration (0~1024)")
grid on

subplot(2,3,4)
plot(t1feedback(1:700), feedback0(1:700,5));
title("Gyro X"+n+"(mapping: 0~1024. -27.925~27.925[m/s^2])")
xlabel("step")
ylabel("gyro (0~1024)")
grid on
%https://cyberbotics.com/doc/reference/gyro
subplot(2,3,5)
plot(t1feedback(1:700), feedback0(1:700,6));
title("Gyro Y"+n+"(mapping: 0~1024. -27.925~27.925[m/s^2])")
xlabel("step")
ylabel("gyro (0~1024)")
grid on

subplot(2,3,6)
plot(t1feedback(1:700), feedback0(1:700,7));
title("Gyro Z"+n+"(mapping: 0~1024. -27.925~27.925[m/s^2])")
xlabel("step")
ylabel("gyro (0~1024)")
grid on

figure;
subplot(2,1,1)
plot(t1walking, walking_pattern0(:,2))
hold on;
plot(t1walking, walking_pattern0(:,3))
%plot(t1walking, walkingpattern1(:,5))
%plot(t1walking, walkingpattern1(:,6))
plot(t1walking, walking_pattern0(:,8))
plot(t1walking, walking_pattern0(:,9))
plot(t1footstep, foot_step0(:,2))
plot(t1footstep, foot_step0(:,3))
legend("CoG Pos X [m]", "CoG Pos Y [m]", "Fixed ZMP Pos X [m]", "Fixed ZMP Pos Y [m]", "ZMP Pos X [m]", "ZMP Pos Y [m]", Location="eastoutside")
xlabel("step")
ylabel("position [m]")
title("CoG & ZMP Trajectory")
grid on

subplot(2,1,2)
plot(walking_pattern0(:,2), walking_pattern0(:,3))
hold on;
plot(walking_pattern0(:,2), walking_pattern0(:,9))
plot(walking_pattern0(:,2), foot_step0(:,3))
legend("CoG Pos [m]", "Fixed ZMP Pos [m]", "ZMP Pos [m]", Location="eastoutside")
xlabel("x-axis position [m]")
ylabel("y-axis position [m]")
grid on

%figure;
%plot(t1jointstates, jointstates1(:,5));
