clear all;clc;
cd(fileparts(mfilename('fullpath')));

CSV_PATH = 'Data/MPXY150Z10'; DogMode = 4;
% CSV_PATH = 'Data/MWXY150Z10'; DogMode = 6;
% CSV_PATH = 'Data/GO2Stairs'; DogMode = 99;
% CSV_PATH = 'Data/GO2Flat'; DogMode = 99;

used_lines = 200000;

data = readmatrix(CSV_PATH);
N = size(data, 1);

fid = fopen(CSV_PATH,'r');
C = textscan(fid, '%s%*[^\n]', N+1, 'Delimiter',',');  % 多读1行，包含表头
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

q16   = data(:, base0 + (0:15) * stride + 6);  % q16：16个电机的q值
dq16  = data(:, base0 + (0:15) * stride + 7);  % dq16：16个电机的dq值
tau16 = data(:, base0 + (0:15) * stride + 8);  % tau16：16个电机的tau值

acc  = data(:, imu0 + (1:3));   % 加速度数据
gyro = data(:, imu0 + (4:6));   % 陀螺仪数据
quat = data(:, imu0 + (7:10));  % 四元数数据

fusion_estimator_mex('reset');
status_ = zeros(200,1,'double');
status_(1) = DogMode;
status_ = fusion_estimator_mex('status',status_);
status_(1) = 2;
status_ = fusion_estimator_mex('status',status_);
status_(1:7) = 1;
status_ = fusion_estimator_mex('status',status_);

odom_log = nan(N, 27);
status_record = zeros(length(status_), N); 

range = 1:1:N;
tic
for k = range
    ts_ms = ts_ms_all(k);

    out = fusion_estimator_mex('step', ts_ms, q16(k,:), dq16(k,:), tau16(k,:), acc(k,:), gyro(k,:), quat(k,:));
    
    status_(1) = 2;
    status_ = fusion_estimator_mex('status',status_);
    status_record(:,k) = status_;

    odom_log(k,:) = [double(ts_ms)/1000, out.XPos, out.YPos, out.ZPos, ...
        out.XVel, out.YVel, out.ZVel, ...
        out.XAcc, out.YAcc, out.ZAcc, ...
        out.RollRad, out.PitchRad, out.YawRad, ...
        out.RollVel, out.PitchVel, out.YawVel,...
        out.RollAcc, out.PitchAcc, out.YawAcc,...
        out.FootfallAverageX, out.FootfallAverageY, out.FootfallAverageYaw, out.LoadedWeight,...
        out.FLFootLanded, out.FRFootLanded, out.RLFootLanded, out.RRFootLanded];

    if mod(k,1000)==0
        fprintf('[%d] t=%.1f x=%.1f y=%.1f z=%.1f r=%.2f p=%.2f y=%.2f\n', ...
            k, odom_log(k,1), odom_log(k,2), odom_log(k,3), odom_log(k,4), odom_log(k,5), odom_log(k,6), odom_log(k,7));
    end
end
toc
fusion_estimator_mex('reset');
clear fusion_estimator_mex
fprintf('[DONE] frames=%d\n', k);

ZoomTime = [0,data(end-1,1)/1000];
ZoomTime = [0,587];

FigureScale = [0 0 900 600];
WordSize = 16;

figure(1); clf; hold on; grid on;
set(gcf,'position',FigureScale);
plot(odom_log(range,1), odom_log(range,2), 'bo')
plot(odom_log(range,1), odom_log(range,3), 'ro')
plot(odom_log(range,1), odom_log(range,4), 'go')
legend('CAPO-X', 'CAPO-Y', 'CAPO-Z','Location','southwest','FontSize', WordSize);
xlabel('Time /s','FontSize', WordSize);
ylabel('Estimated State /m','FontSize', WordSize);
title('CAPO Method XYZ Accuracy Check','FontSize', WordSize);

% figure(2); clf; hold on; grid on;
% plot(odom_log(range,1), odom_log(range,5), 'bo')
% plot(odom_log(range,1), odom_log(range,6), 'ro')
% plot(odom_log(range,1), odom_log(range,7), 'go')
% legend("vX","vY","vZ")
% 
% figure(8); clf;
% % --- Px (i=0) ---
% subplot(3,1,1); hold on; grid on;
% title("Foot points XYZ in body frame")
% plot(odom_log(range,1), status_record(101 + 0*3 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 1*3 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 2*3 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 3*3 + 0, range), 'LineWidth',1.1);
% ylabel('X (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3');
% % --- Py (i=1) ---
% subplot(3,1,2); hold on; grid on;
% plot(odom_log(range,1), status_record(101 + 0*3 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 1*3 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 2*3 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 3*3 + 1, range), 'LineWidth',1.1);
% ylabel('Y (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3');
% % --- Pz (i=2) ---
% subplot(3,1,3); 
% hold on; grid on;
% plot(odom_log(range,1), status_record(101 + 0*3 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 1*3 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 2*3 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(101 + 3*3 + 2, range), 'LineWidth',1.1);
% xlabel('t (s)'); ylabel('Z (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3');
% 
% figure(9); clf;
% subplot(4,1,1); hold on; grid on;
% title("Foot effort XYZ in body frames")
% plot(odom_log(range,1), status_record(113 + 0*6 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 1*6 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 2*6 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 3*6 + 0, range), 'LineWidth',1.1);
% plot(odom_log(range,1), abs(status_record(113 + 0*6 + 0, range))+abs(status_record(113 + 1*6 + 0, range))+abs(status_record(113 + 2*6 + 0, range))+abs(status_record(113 + 3*6 + 0, range)), 'LineWidth',1.1);
% ylabel('Fx (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3','Total',Location='best');
% % --- Ty (i=4) ---
% subplot(4,1,2); hold on; grid on;
% plot(odom_log(range,1), status_record(113 + 0*6 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 1*6 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 2*6 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 3*6 + 1, range), 'LineWidth',1.1);
% plot(odom_log(range,1), abs(status_record(113 + 0*6 + 1, range))+abs(status_record(113 + 1*6 + 1, range))+abs(status_record(113 + 2*6 + 1, range))+abs(status_record(113 + 3*6 + 1, range)), 'LineWidth',1.1);
% ylabel('Fy (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3','Total',Location='best');
% % --- Tz (i=5) ---
% subplot(2,1,2); hold on; grid on;
% plot(odom_log(range,1), status_record(113 + 0*6 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 1*6 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 2*6 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 3*6 + 2, range), 'LineWidth',1.1);
% plot(odom_log(range,1), abs(status_record(113 + 0*6 + 2, range))+abs(status_record(113 + 1*6 + 2, range))+abs(status_record(113 + 2*6 + 2, range))+abs(status_record(113 + 3*6 + 2, range)), 'LineWidth',1.1);
% xlabel('t (s)'); ylabel('Fz (body)');
% xlim(ZoomTime)
% ylim([-400,0])
% legend('Leg0','Leg1','Leg2','Leg3','Total',Location='best');
% 
% figure(10); clf;
% subplot(3,1,1); hold on; grid on;
% title("Foot torque XYZ in body frames")
% plot(odom_log(range,1), status_record(113 + 0*6 + 3, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 1*6 + 3, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 2*6 + 3, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 3*6 + 3, range), 'LineWidth',1.1);
% plot(odom_log(range,1), odom_log(range,12) * 1000, 'LineWidth',1.1);
% plot(odom_log(range,1), abs(status_record(113 + 0*6 + 3, range))+abs(status_record(113 + 1*6 + 3, range))+abs(status_record(113 + 2*6 + 3, range))+abs(status_record(113 + 3*6 + 3, range)), 'LineWidth',1.1);
% ylabel('Tx (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3','pitch','Total',Location='best');
% % --- Ty (i=4) ---
% subplot(3,1,2); hold on; grid on;
% plot(odom_log(range,1), status_record(113 + 0*6 + 4, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 1*6 + 4, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 2*6 + 4, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 3*6 + 4, range), 'LineWidth',1.1);
% plot(odom_log(range,1), odom_log(range,11) * 10000, 'LineWidth',1.1);
% plot(odom_log(range,1), abs(status_record(113 + 0*6 + 4, range))+abs(status_record(113 + 1*6 + 4, range))+abs(status_record(113 + 2*6 + 4, range))+abs(status_record(113 + 3*6 + 4, range)), 'LineWidth',1.1);
% ylabel('Ty (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3','roll','Total',Location='best');
% % --- Tz (i=5) ---
% subplot(3,1,3); hold on; grid on;
% plot(odom_log(range,1), status_record(113 + 0*6 + 5, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 1*6 + 5, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 2*6 + 5, range), 'LineWidth',1.1);
% plot(odom_log(range,1), status_record(113 + 3*6 + 5, range), 'LineWidth',1.1);
% plot(odom_log(range,1), odom_log(range,12) * 1000, 'LineWidth',1.1);
% plot(odom_log(range,1), abs(status_record(113 + 0*6 + 5, range))+abs(status_record(113 + 1*6 + 5, range))+abs(status_record(113 + 2*6 + 5, range))+abs(status_record(113 + 3*6 + 5, range)), 'LineWidth',1.1);
% xlabel('t (s)'); ylabel('Tz (body)');
% xlim(ZoomTime)
% legend('Leg0','Leg1','Leg2','Leg3','pitch','Total',Location='best');