clear;

load WPG_log.dat;

[a, b] = size(WPG_log);

t = 1:a;

plot(t, WPG_log)

legend("CoG Position X", "CoG Position Y", "CoG Velocity X", "CoG Velocity Y")
xlabel("time")