clear;

load WPG_log.dat;

[a, b] = size(WPG_log);

t = 1:a;

plot(t, WPG_log)

legend("CoG Position X", "CoG Position Y", "CoG Velocity X", "CoG Velocity Y", "p_x_fix", "p_y_fix", "LandingPosition_X", "LandingPosition_Y")
xlabel("time")

% reference https://jp.mathworks.com/help/matlab/data_analysis/plotting-data.html