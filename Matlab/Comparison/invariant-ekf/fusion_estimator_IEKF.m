clear all;
cd(fileparts(mfilename('fullpath')));

CSV_PATH = '../../Data/MPXY150Z10';  DogMode = 4; ContactThreshold = 20; ZoomTime = [0,410];
% CSV_PATH = '../../Data/GO2Stairs';  DogMode = 99; ContactThreshold = 50; ZoomTime = [0,98];
% CSV_PATH = '../../Data/GO2Flat';  DogMode = 99; ContactThreshold = 50; ZoomTime = [0,140];

used_lines = 200000;

data = readmatrix(CSV_PATH);
N = size(data, 1);

fid = fopen(CSV_PATH, 'r');
C = textscan(fid, '%s%*[^\n]', N+1, 'Delimiter', ',');
fclose(fid);

time_strs = string(C{1}(2:end));
pat = '\.(\d{1,3})(?!\d)';
time_strs_fix = regexprep(time_strs, pat, '.${pad($1,3,"left","0")}');

t0 = datetime(time_strs_fix(1), 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
t  = datetime(time_strs_fix,   'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
ts_ms_all = int64(round(milliseconds(t - t0)));

data(:,1) = double(ts_ms_all);

N = min(size(data,1), used_lines);
data = data(1:N,:);
ts_ms_all = ts_ms_all(1:N);

base0     = 2;
stride    = 13;
motor_num = 16;
imu0      = base0 + motor_num * stride;

q16   = data(:, base0 + (0:15) * stride + 6);
dq16  = data(:, base0 + (0:15) * stride + 7);
tau16 = data(:, base0 + (0:15) * stride + 8);

acc  = data(:, imu0 + (1:3));
gyro = data(:, imu0 + (4:6));
quat = data(:, imu0 + (7:10));

inekf_legged_core_mex('reset');

status_ = zeros(200,1,'double');
status_(1) = DogMode;
inekf_legged_core_mex('status', status_);

status_ = zeros(200,1,'double');
status_(1) = 1;      % set
status_(11) = ContactThreshold;   % contact threshold，对应 C++ status[8]
inekf_legged_core_mex('status', status_);

odom_log = nan(N, 4);

range = 1:N;
tic
for k = range
    ts_ms = ts_ms_all(k);

    out = inekf_legged_core_mex('step', ...
        ts_ms, ...
        q16(k,:), ...
        dq16(k,:), ...
        tau16(k,:), ...
        acc(k,:), ...
        gyro(k,:), ...
        quat(k,:));

    odom_log(k,:) = [double(ts_ms)/1000, out.XPos, out.YPos, out.ZPos];

    if mod(k,1000) == 0
        fprintf('[%d] t=%.3f X=%.4f Y=%.4f Z=%.4f\n', ...
            k, odom_log(k,1), odom_log(k,2), odom_log(k,3), odom_log(k,4));
    end
end
toc

inekf_legged_core_mex('reset');
clear inekf_legged_core_mex

fprintf('[DONE] frames=%d\n', k);

% figure(1); clf; 

FigureScale = [0 0 900 600];
WordSize = 16;
hold on; grid on;
plot(odom_log(range,1), -odom_log(range,2), 'b-');
plot(odom_log(range,1), -odom_log(range,3), 'r-');
plot(odom_log(range,1), odom_log(range,4), 'g-');
legend('CAPO-X', 'CAPO-Y', 'CAPO-Z','InEKF-X', 'InEKF-Y', 'InEKF-Z','Location','northeast','FontSize', WordSize);
title('Accuracy Comparision on 3D Path between CAPO and InEKF','FontSize', WordSize);
xlim(ZoomTime)