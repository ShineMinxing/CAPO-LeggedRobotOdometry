clear all;clc;
cd(fileparts(mfilename('fullpath')));

% CSV_PATH = 'Data/GO2Flat'; DogMode = 98;
CSV_PATH = 'Data/GO2Stairs'; DogMode = 98;
% CSV_PATH = 'Data/MP_XY150Z10'; DogMode = 140;
% CSV_PATH = 'Data/MW_XY150Z10'; DogMode = 141;

used_lines = 300000;

data = readmatrix(CSV_PATH);
N = size(data, 1);

fid = fopen(CSV_PATH,'r');
C = textscan(fid, '%s%*[^\n]', N+1, 'Delimiter',',');  % ¶à¶Á1ÐÐ£¬°üº¬±íÍ·
fclose(fid);
time_strs = string(C{1}(2:end));
pat = '\.(\d{1,3})(?!\d)';
time_strs_fix = regexprep(time_strs, pat, '.${pad($1,3,"left","0")}');
t0 = datetime(time_strs_fix(1), 'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
t  = datetime(time_strs_fix,   'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
ts_ms_all = int64(round(milliseconds(t - t0)));
data(:,1) = ts_ms_all;

N = min(size(data, 1), used_lines);
data = data(1:N,:);

base0     = 2;
stride    = 13;
motor_num = 16;
imu0      = base0 + motor_num * stride;

q16   = data(:, base0 + (0:15) * stride + 6);  % q16 with 16 motors angle
dq16  = data(:, base0 + (0:15) * stride + 7);  % dq16 with 16 motors angular velocity
tau16 = data(:, base0 + (0:15) * stride + 8);  % tau16 with 16 motors torque

acc  = data(:, imu0 + (1:3));   % Acceleration data from imu
gyro = data(:, imu0 + (4:6));   % Gyroscope data from imu
quat = data(:, imu0 + (7:10));  % Quaternion data from imu

fusion_estimator_mex('reset');
status_ = zeros(100,1,'double');
status_(1) = DogMode;
status_ = fusion_estimator_mex('status',status_);
status_(1) = 2;
status_ = fusion_estimator_mex('status',status_);
status_(1:7) = 1;
status_ = fusion_estimator_mex('status',status_);

odom_log = nan(N, 27);

range = 1:1:N;
tic
for k = range
    if(k==10)
        status_(1) = 3;
        status_ = fusion_estimator_mex('status',status_);
    end

    ts_ms = ts_ms_all(k);

    out = fusion_estimator_mex('step', ts_ms, q16(k,:), dq16(k,:), tau16(k,:), acc(k,:), gyro(k,:), quat(k,:));

    odom_log(k,:) = [double(ts_ms)/1000, double(out.PositionXYZ([1,4,7])), double(out.PositionXYZ([2,5,8])), double(out.PositionXYZ([3,6,9])),...
        double(out.OrientationRPY([1,4,7])), double(out.OrientationRPY([2,5,8])), double(out.OrientationRPY([3,6,9])),...
        double(out.FootfallAverage(:).'), double(out.DogWeight), double(out.FootLandedProbability(:).')];

    if mod(k,1000)==0
        fprintf('[%d] t=%.1f x=%.1f y=%.1f z=%.1f r=%.2f p=%.2f y=%.2f\n', ...
            k, odom_log(k,1), odom_log(k,2), odom_log(k,3), odom_log(k,4), odom_log(k,11), odom_log(k,12), odom_log(k,13));
    end
end
toc
fusion_estimator_mex('reset');
clear fusion_estimator_mex
fprintf('[DONE] frames=%d\n', k);

ZoomTime = [0,data(end-1,1)/1000];
% ZoomTime = [10.6,11.6];

WordSize = 25;

DP0 = 21;
leg_names = {'FL','FR','RL','RR'};
xyz_names = {'X','Y','Z'};
node_names = {'hip','thigh','calf','foot'};

t_plot = odom_log(range,1);

figure(1); clf;

tabs_state = uitabgroup;

tab_pos = uitab(tabs_state, 'Title', 'Position');
tab_ori = uitab(tabs_state, 'Title', 'Orientation');
tab_vel_acc = uitab(tabs_state, 'Title', 'Vel_Acc');
tab_ang = uitab(tabs_state, 'Title', 'AngVel_Acc');

t_plot = odom_log(range,1);

% ================= Position =================
subplot(1,1,1,'Parent',tab_pos);
hold on; grid on;

plot(t_plot, odom_log(range,2),'r');
plot(t_plot, odom_log(range,3),'g');
plot(t_plot, odom_log(range,4),'b');

xlim(ZoomTime);
legend('X','Y','Z','Location','best','FontSize',WordSize);
xlabel('Time /s','FontSize',WordSize);
ylabel('Position /m','FontSize',WordSize);
title('Position','FontSize',WordSize);


% ================= Orientation =================
subplot(1,1,1,'Parent',tab_ori);
hold on; grid on;

plot(t_plot, odom_log(range,11),'r');
plot(t_plot, odom_log(range,12),'g');
plot(t_plot, odom_log(range,13),'b');

xlim(ZoomTime);
legend('Roll','Pitch','Yaw','Location','best','FontSize',WordSize);
xlabel('Time /s','FontSize',WordSize);
ylabel('Angle /rad','FontSize',WordSize);
title('Orientation','FontSize',WordSize);

% ================= Velocity + Acceleration =================
subplot(1,1,1,'Parent',tab_vel_acc);
hold on; grid on;

% velocity ÊµÏß
plot(t_plot, odom_log(range,5),'r-');
plot(t_plot, odom_log(range,6),'g-');
plot(t_plot, odom_log(range,7),'b-');

% acceleration ÐéÏß
plot(t_plot, odom_log(range,8), '--', 'Color', [1 0.3 0.3]);
plot(t_plot, odom_log(range,9), '--', 'Color', [0.3 1 0.3]);
plot(t_plot, odom_log(range,10), '--', 'Color', [0.3 0.3 1]);


xlim(ZoomTime);
legend('vX','vY','vZ','aX','aY','aZ',...
    'Location','best','FontSize',WordSize);

xlabel('Time /s','FontSize',WordSize);
ylabel('Velocity / Acceleration','FontSize',WordSize);
title('Velocity and Acceleration','FontSize',WordSize);

% ================= Angular Velocity + Angular Acceleration =================
subplot(1,1,1,'Parent',tab_ang);
hold on; grid on;

% angular velocity ÊµÏß
plot(t_plot, odom_log(range,14),'r-');
plot(t_plot, odom_log(range,15),'g-');
plot(t_plot, odom_log(range,16),'b-');

% angular acceleration ÐéÏß
plot(t_plot, odom_log(range,17)/5,'r--');
plot(t_plot, odom_log(range,18)/5,'g--');
plot(t_plot, odom_log(range,19)/5,'b--');

xlim(ZoomTime);
legend('RollVel','PitchVel','YawVel',...
       'RollAcc','PitchAcc','YawAcc',...
       'Location','best','FontSize',WordSize);

xlabel('Time /s','FontSize',WordSize);
ylabel('Angular State','FontSize',WordSize);
title('Angular Velocity and Acceleration','FontSize',WordSize);