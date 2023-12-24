clear all;
close all;

load("../1_2023-12-25_01-36-26/feedback1.dat");
load("../1_2023-12-25_01-36-26/footstep1.dat");
load("../1_2023-12-25_01-36-26/jointstates1.dat");
load("../1_2023-12-25_01-36-26/walkingpattern1.dat");
load("../1_2023-12-25_01-36-26/walkingstabilization1.dat");

[nf, pf] = size(feedback1);
[nw, pw] = size(walkingpattern1);
[ns, ps] = size(footstep1);
[nj, pj] = size(jointstates1);

t1feedback = 1:nf;
t1walking = 1:nw;
t1footstep = 1:ns;
for t = 1:ns
  footstep1(t,3) = footstep1(t,3) - 0.037;
end
t1jointstates = 1:nj;

figure;
subplot(2,3,1)
plot(t1feedback(1:700), feedback1(1:700,2));
title("Accelerometer X [m/s^2]")
xlabel("step")
ylabel("acceleration [m/s^2]")
grid on
%https://cyberbotics.com/doc/reference/accelerometer
subplot(2,3,2)
plot(t1feedback(1:700), feedback1(1:700,3));
title("Accelerometer Y [m/s^2]")
xlabel("step")
ylabel("acceleration [m/s^2]")
grid on

subplot(2,3,3)
plot(t1feedback(1:700), feedback1(1:700,4));
title("Accelerometer Z [m/s^2]")
xlabel("step")
ylabel("acceleration [m/s^2]")
grid on

subplot(2,3,4)
plot(t1feedback(1:700), feedback1(1:700,5));
title("Gyro X [rad/s]")
xlabel("step")
ylabel("angular velocity [rad/s]")
grid on
%https://cyberbotics.com/doc/reference/gyro
subplot(2,3,5)
plot(t1feedback(1:700), feedback1(1:700,6));
title("Gyro Y [rad/s]")
xlabel("step")
ylabel("angular velocity [rad/s]")
grid on

subplot(2,3,6)
plot(t1feedback(1:700), feedback1(1:700,7));
title("Gyro Z [rad/s]")
xlabel("step")
ylabel("angular velocity [rad/s]")
grid on

figure;
subplot(2,1,1)
plot(t1walking, walkingpattern1(:,2))
hold on;
plot(t1walking, walkingpattern1(:,3))
%plot(t1walking, walkingpattern1(:,5))
%plot(t1walking, walkingpattern1(:,6))
plot(t1walking, walkingpattern1(:,8))
plot(t1walking, walkingpattern1(:,9))
plot(t1footstep, footstep1(:,2))
plot(t1footstep, footstep1(:,3))
legend("CoG Pos X [m]", "CoG Pos Y [m]", "Fixed ZMP Pos X [m]", "Fixed ZMP Pos Y [m]", "ZMP Pos X [m]", "ZMP Pos Y [m]", Location="eastoutside")
xlabel("step")
ylabel("position [m]")
title("CoG & ZMP Trajectory")
grid on

subplot(2,1,2)
plot(walkingpattern1(:,2), walkingpattern1(:,3))
hold on;
plot(walkingpattern1(:,2), walkingpattern1(:,9))
plot(walkingpattern1(:,2), footstep1(:,3))
legend("CoG Pos [m]", "Fixed ZMP Pos [m]", "ZMP Pos [m]", Location="eastoutside")
xlabel("x-axis position [m]")
ylabel("y-axis position [m]")
grid on

%figure;
%plot(t1jointstates, jointstates1(:,5));
