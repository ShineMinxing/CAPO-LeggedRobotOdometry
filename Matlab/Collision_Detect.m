if ~exist('odom_log','var') || ~exist('status_record','var')
    error(['缺少 odom_log 或 status_record。', newline, ...
           '请先运行 fusion_estimator.m，并保留工作区变量，不要 clear 工作区。']);
end

if ~exist('range','var') || isempty(range)
    range = 1:size(odom_log,1);
end

if ~exist('WordSize','var') || isempty(WordSize)
    WordSize = 16;
end

% ================= 碰撞判定参数 =================
acc_threshold = 0.25;           % 水平姿态、RobotTypePar=1时的基础阈值

% 可在运行本脚本前于工作区手动设置RobotTypePar；
% 若没有设置，则默认使用1。
if ~exist('RobotTypePar','var') || isempty(RobotTypePar)
    RobotTypePar = 1;
end

% Roll与Pitch绝对值之和超过10度后，阈值连续线性提高：
% threshold_factor = max((tilt_deg - 10)/15 + 1, 1)
tilt_start_deg = 10;
tilt_step_deg = 10;

collision_dead_time = 1.0;     % 确认一次碰撞后，后续屏蔽时间 /s

velocity_average_frames = 10;  % 撞击前10帧平均速度，不包含当前帧
velocity_min = 0.2;            % 速度过小时，速度方向不可信 /m/s

% abs(YawAcc)只用于放宽“速度反向与ImpactAcc”的允许夹角
angle_base_deg = 30;           % YawAcc=0时允许夹角
angle_yaw_gain_deg = 5;        % 每1 rad/s^2角加速度增加的允许角度
angle_max_deg = 75;            % 允许夹角上限

% 碰撞点方向距离正前、正右、正后、正左不超过该角度时，
% 认为同时撞到该面的两条腿。
face_snap_deg = 15;

% ================= 腿编号和位置 =================
% 身体坐标系：+X向前，+Y向左
%
% Leg0：左前  FL  (+X,+Y)
% Leg1：右前  FR  (+X,-Y)
% Leg2：左后  RL  (-X,+Y)
% Leg3：右后  RR  (-X,-Y)
leg_xy = [
     1,  1
     1, -1
    -1,  1
    -1, -1
];

% collision_face_code 编码：
%   0  = ALeg0
%   1  = ALeg1
%   2  = ALeg2
%   3  = ALeg3
%   10 = ALeg01，正前面
%   13 = ALeg13，正右面
%   32 = ALeg32，正后面
%   20 = ALeg20，正左面
%
% collision_event_record 每行：
%   [原始帧号, 时间/s, collision_face_code]

% ================= 绘图参数 =================
acc_plot_scale = 1/10;

yaw_acc_min = 0.00;
yaw_arc_gain = 0.15;
yaw_arc_max = 1.5*pi;
yaw_arc_radius = 0.25;
yaw_arc_point_num = 30;

leg_color = [
    1.00 0.00 1.00   % Leg0
    0.00 1.00 1.00   % Leg1
    0.85 0.65 0.00   % Leg2
    0.00 0.65 0.00   % Leg3
];

% Legend按照0 -> 1 -> 3 -> 2的机体周向顺序排列
acc_category_code = [0, 10, 1, 13, 3, 32, 2, 20];
acc_category_name = { ...
    'ALeg0','ALeg01','ALeg1','ALeg13', ...
    'ALeg3','ALeg32','ALeg2','ALeg20'};

acc_category_color = [
    leg_color(1,:)
    leg_color(1,:)
    leg_color(2,:)
    leg_color(2,:)
    leg_color(4,:)
    leg_color(4,:)
    leg_color(3,:)
    leg_color(3,:)
];

acc_category_style = {'--','-','--','-','--','-','--','-'};

% ================= 数据检查 =================
range = range(:);
range = range(isfinite(range));
range = range(range == fix(range));
range = range(range >= 1);
range = range(range <= size(odom_log,1));
range = range(range <= size(status_record,2));

if isempty(range)
    error('range 中没有有效的数据序号。');
end

if any(diff(range) <= 0)
    error('range 必须严格递增，不能包含重复或倒序索引。');
end

if size(odom_log,2) < 23
    error('odom_log 列数不足，需要至少包含第23列机器狗质量。');
end

if size(status_record,1) < 31
    error(['status_record 行数不足，无法读取四条腿的XY足端等效力。', newline, ...
           '请先完整运行 fusion_estimator.m。']);
end

if velocity_average_frames < 1 ...
        || velocity_average_frames ~= fix(velocity_average_frames)
    error('velocity_average_frames 必须是正整数。');
end

if face_snap_deg < 0 || face_snap_deg >= 45
    error('face_snap_deg 必须位于[0,45)度。');
end

if ~isscalar(RobotTypePar) ...
        || ~isfinite(RobotTypePar) ...
        || RobotTypePar <= 0
    error('RobotTypePar 必须是大于0的有限标量。');
end

if tilt_start_deg < 0 || tilt_step_deg <= 0
    error('姿态阈值参数设置无效。');
end

sample_id = double(range);
time_sec = odom_log(range,1);

if any(~isfinite(time_sec))
    error('odom_log(:,1) 中存在无效时间。');
end

if any(diff(time_sec) < 0)
    error('odom_log(:,1) 时间不是单调递增。');
end

data_num = numel(range);

roll = odom_log(range,11);
pitch = odom_log(range,12);
yaw = odom_log(range,13);
yaw_acc = odom_log(range,19);
dog_mass = odom_log(range,23);

vel_x_world = odom_log(range,5);
vel_y_world = odom_log(range,6);

imu_acc_x_world = odom_log(range,8);
imu_acc_y_world = odom_log(range,9);

% ================= 世界坐标系足端合力加速度 =================
% 四条腿足端力先在世界系按分量求和。
force_sum_x_world = ...
    sum(status_record([21,24,27,30],range),1).';

force_sum_y_world = ...
    sum(status_record([22,25,28,31],range),1).';

mass_valid = isfinite(dog_mass) & dog_mass > 0;

facc_x_world = nan(data_num,1);
facc_y_world = nan(data_num,1);

facc_x_world(mass_valid) = ...
    force_sum_x_world(mass_valid) ./ dog_mass(mass_valid);

facc_y_world(mass_valid) = ...
    force_sum_y_world(mass_valid) ./ dog_mass(mass_valid);

% 当前足端力直接计算的FAcc方向，与足端实际给身体的加速度反向：
% ImpactAcc = IMU Acc + FAcc。
impact_acc_x_world = imu_acc_x_world + facc_x_world;
impact_acc_y_world = imu_acc_y_world + facc_y_world;

impact_acc_norm = hypot(impact_acc_x_world,impact_acc_y_world);
impact_acc_norm_plot = impact_acc_norm * acc_plot_scale;

% ================= Roll/Pitch动态加速度阈值 =================
% odom_log(:,11:12)单位为rad。
tilt_abs_sum_deg = rad2deg(abs(roll) + abs(pitch));
attitude_valid = isfinite(tilt_abs_sum_deg);

tilt_threshold_factor = max( ...
    (tilt_abs_sum_deg - tilt_start_deg) / tilt_step_deg + 1, ...
    1);

acc_threshold_dynamic = ...
    acc_threshold * RobotTypePar .* tilt_threshold_factor;

% ================= 撞击前10帧世界系平均速度 =================
% 模长和夹角在同一世界坐标系中计算，不需要Yaw旋转。
vel_mean_x_world = nan(data_num,1);
vel_mean_y_world = nan(data_num,1);

for ii = 1:data_num
    global_id = range(ii);

    first_id = global_id - velocity_average_frames;
    last_id = global_id - 1;

    if first_id < 1
        continue;
    end

    prev_vx = odom_log(first_id:last_id,5);
    prev_vy = odom_log(first_id:last_id,6);

    if any(~isfinite(prev_vx)) || any(~isfinite(prev_vy))
        continue;
    end

    vel_mean_x_world(ii) = mean(prev_vx);
    vel_mean_y_world(ii) = mean(prev_vy);
end

vel_mean_norm = hypot(vel_mean_x_world,vel_mean_y_world);

% ================= 世界系速度反向与ImpactAcc夹角 =================
reverse_vel_x_world = -vel_mean_x_world;
reverse_vel_y_world = -vel_mean_y_world;

angle_error_rad = nan(data_num,1);

angle_valid = ...
    isfinite(reverse_vel_x_world) ...
    & isfinite(reverse_vel_y_world) ...
    & isfinite(impact_acc_x_world) ...
    & isfinite(impact_acc_y_world) ...
    & isfinite(yaw_acc) ...
    & mass_valid ...
    & vel_mean_norm >= velocity_min ...
    & impact_acc_norm > 0;

angle_cross = ...
    reverse_vel_x_world .* impact_acc_y_world ...
    - reverse_vel_y_world .* impact_acc_x_world;

angle_dot = ...
    reverse_vel_x_world .* impact_acc_x_world ...
    + reverse_vel_y_world .* impact_acc_y_world;

angle_error_rad(angle_valid) = atan2( ...
    abs(angle_cross(angle_valid)), ...
    angle_dot(angle_valid));

angle_allow_deg = min( ...
    angle_base_deg ...
    + angle_yaw_gain_deg .* abs(yaw_acc), ...
    angle_max_deg);

angle_allow_rad = deg2rad(angle_allow_deg);

% ================= 真实撞击判定 =================
% 阈值 = 基础阈值 × RobotTypePar × Roll/Pitch姿态倍率。
acc_candidate = ...
    attitude_valid ...
    & impact_acc_norm_plot > acc_threshold_dynamic;

tempTrue_raw = ...
    acc_candidate ...
    & angle_valid ...
    & angle_error_rad <= angle_allow_rad;

% ================= 碰撞时间屏蔽 =================
tempTrue = false(size(tempTrue_raw));
last_collision_time = -inf;

for ii = 1:data_num
    if tempTrue_raw(ii) ...
            && time_sec(ii) - last_collision_time >= collision_dead_time
        tempTrue(ii) = true;
        last_collision_time = time_sec(ii);
    end
end

collision_id = find(tempTrue);

% ================= 只在判腿前旋转ImpactAcc =================
cy = cos(yaw);
sy = sin(yaw);

impact_acc_x_body = ...
    cy .* impact_acc_x_world + sy .* impact_acc_y_world;

impact_acc_y_body = ...
   -sy .* impact_acc_x_world + cy .* impact_acc_y_world;

leg_dir = leg_xy ./ vecnorm(leg_xy,2,2);

collision_face_code = -ones(data_num,1);

for jj = 1:numel(collision_id)
    ii = collision_id(jj);

    a_body = [impact_acc_x_body(ii),impact_acc_y_body(ii)];
    a_norm = norm(a_body);

    if ~isfinite(a_norm) || a_norm <= 0
        continue;
    end

    % 撞击点方向与外部冲击加速度方向相反。
    contact_dir = -a_body / a_norm;
    contact_angle_deg = atan2d(contact_dir(2),contact_dir(1));

    if abs(wrap_angle_deg(contact_angle_deg -   0)) <= face_snap_deg
        collision_face_code(ii) = 10;  % 正前：Leg0 + Leg1
    elseif abs(wrap_angle_deg(contact_angle_deg +  90)) <= face_snap_deg
        collision_face_code(ii) = 13;  % 正右：Leg1 + Leg3
    elseif abs(wrap_angle_deg(contact_angle_deg - 180)) <= face_snap_deg
        collision_face_code(ii) = 32;  % 正后：Leg3 + Leg2
    elseif abs(wrap_angle_deg(contact_angle_deg -  90)) <= face_snap_deg
        collision_face_code(ii) = 20;  % 正左：Leg2 + Leg0
    else
        leg_score = leg_dir * contact_dir.';
        [~,best_leg_index] = max(leg_score);
        collision_face_code(ii) = best_leg_index - 1;
    end
end

valid_collision_id = ...
    collision_id(collision_face_code(collision_id) >= 0);

% 记录全部碰撞事件。
collision_event_record = [
    sample_id(valid_collision_id), ...
    time_sec(valid_collision_id), ...
    collision_face_code(valid_collision_id)
];

draw_local_id = valid_collision_id(1:end);

% ================= 基础曲线和固定图例 =================
fig = figure(2);
clf(fig);

ax = axes('Parent',fig);
hold(ax,'on');
grid(ax,'on');

h_x = plot(ax,sample_id,odom_log(range,2), ...
    'b','LineWidth',1.2);

h_y = plot(ax,sample_id,odom_log(range,3), ...
    'b-.','LineWidth',1.2);

h_vel = plot(ax,nan,nan,'r-.');

h_yaw = plot(ax,nan,nan,'k-');

h_acc_category = gobjects(numel(acc_category_code),1);

for category_id = 1:numel(acc_category_code)
    h_acc_category(category_id) = plot(ax,nan,nan, ...
        acc_category_style{category_id}, ...
        'Color',acc_category_color(category_id,:));
end

yline(ax,0,'Color',[0.5 0.5 0.5], ...
    'LineStyle',':', ...
    'HandleVisibility','off');

x_margin = max(1,0.01*(sample_id(end)-sample_id(1)+1));

xlim(ax,[sample_id(1)-x_margin, sample_id(end)+x_margin]);
ylim(ax,[-1.5,1.5]);

xlabel(ax,'Sample Index','FontSize',WordSize);
ylabel(ax,'Position / Collision Direction','FontSize',WordSize);
title(ax,'Collision Detection','FontSize',WordSize);

drawnow;

% ================= 屏幕比例补偿 =================
% 当前绘图直接使用世界系向量：
%   向上 = 世界+X
%   向左 = 世界+Y
%   逆时针 = +YawAcc
old_units = ax.Units;
ax.Units = 'pixels';
ax_pos = ax.Position;
ax.Units = old_units;

x_range = diff(xlim(ax));
y_range = diff(ylim(ax));

if ax_pos(3) <= 0 || ax_pos(4) <= 0 || x_range <= 0 || y_range <= 0
    error('坐标轴尺寸或显示范围无效。');
end

x_data_per_y_visual = ...
    x_range * ax_pos(4) / (y_range * ax_pos(3));

if ~isfinite(x_data_per_y_visual) || x_data_per_y_visual <= 0
    error('坐标轴显示比例计算失败。');
end

% ================= 世界系平均速度线段 =================
if ~isempty(draw_local_id)
    vel_up = vel_mean_x_world(draw_local_id);
    vel_left = vel_mean_y_world(draw_local_id);

    vel_dx = -vel_left * x_data_per_y_visual;
    vel_dy =  vel_up;

    [vel_line_x,vel_line_y] = make_line_segments( ...
        sample_id(draw_local_id), ...
        zeros(size(draw_local_id)), ...
        vel_dx,vel_dy);

    plot(ax,vel_line_x,vel_line_y,'r-.', ...
        'HandleVisibility','off');
end

% ================= 世界系ImpactAcc分类线段 =================
for category_id = 1:numel(acc_category_code)
    code_now = acc_category_code(category_id);

    category_collision_id = draw_local_id( ...
        collision_face_code(draw_local_id) == code_now);

    if isempty(category_collision_id)
        continue;
    end

    acc_up = ...
        impact_acc_x_world(category_collision_id) * acc_plot_scale;

    acc_left = ...
        impact_acc_y_world(category_collision_id) * acc_plot_scale;

    acc_dx = -acc_left * x_data_per_y_visual;
    acc_dy =  acc_up;

    [acc_line_x,acc_line_y] = make_line_segments( ...
        sample_id(category_collision_id), ...
        zeros(size(category_collision_id)), ...
        acc_dx,acc_dy);

    plot(ax,acc_line_x,acc_line_y, ...
        acc_category_style{category_id}, ...
        'Color',acc_category_color(category_id,:), ...
        'HandleVisibility','off');
end

% ================= Yaw角加速度圆弧 =================
arc_local_id = draw_local_id( ...
    isfinite(yaw_acc(draw_local_id)) ...
    & abs(yaw_acc(draw_local_id)) >= yaw_acc_min);

arc_x_radius = yaw_arc_radius * x_data_per_y_visual;
arc_y_radius = yaw_arc_radius;

if ~isempty(arc_local_id)
    arc_count = numel(arc_local_id);

    arc_x = nan(yaw_arc_point_num+1,arc_count);
    arc_y = nan(yaw_arc_point_num+1,arc_count);

    for jj = 1:arc_count
        ii = arc_local_id(jj);

        direction_sign = sign(yaw_acc(ii));

        arc_angle = min( ...
            abs(yaw_acc(ii))*yaw_arc_gain, ...
            yaw_arc_max);

        theta = linspace( ...
            pi/2, ...
            pi/2 + direction_sign*arc_angle, ...
            yaw_arc_point_num);

        arc_x(1:yaw_arc_point_num,jj) = ...
            sample_id(ii) + arc_x_radius*cos(theta);

        arc_y(1:yaw_arc_point_num,jj) = ...
            arc_y_radius*sin(theta);
    end

    plot(ax,arc_x(:),arc_y(:),'k-', ...
        'HandleVisibility','off');
end

legend( ...
    [h_x,h_y,h_vel,h_yaw,h_acc_category.'], ...
    [{'X','Y','Vel','aYaw'},acc_category_name], ...
    'FontSize',WordSize);

% ================= 检测结果输出 =================
fprintf( ...
    ['[Collision Plot] RobotTypePar=%.3f, Acc candidates=%d, ', ...
     'angle matched=%d, after %.3fs suppression=%d\n'], ...
    RobotTypePar, ...
    nnz(acc_candidate), ...
    nnz(tempTrue_raw), ...
    collision_dead_time, ...
    size(collision_event_record,1));

fprintf(['[Collision Record] columns = ', ...
    '[frame, time_s, face_code]\n']);

for jj = 1:size(collision_event_record,1)
    ii = valid_collision_id(jj);

    fprintf( ...
        ['[Collision] frame=%d t=%.3f code=%d ', ...
         '|ImpactAcc|/10=%.3f threshold=%.3f ', ...
         'tilt=%.1fdeg factor=%.3f angle=%.1fdeg ', ...
         'allow=%.1fdeg |YawAcc|=%.3f\n'], ...
        collision_event_record(jj,1), ...
        collision_event_record(jj,2), ...
        collision_event_record(jj,3), ...
        impact_acc_norm_plot(ii), ...
        acc_threshold_dynamic(ii), ...
        tilt_abs_sum_deg(ii), ...
        tilt_threshold_factor(ii), ...
        rad2deg(angle_error_rad(ii)), ...
        angle_allow_deg(ii), ...
        abs(yaw_acc(ii)));
end


function angle_deg = wrap_angle_deg(angle_deg)

    angle_deg = mod(angle_deg + 180,360) - 180;
end


function [line_x,line_y] = make_line_segments(x0,y0,dx,dy)

    x0 = x0(:);
    y0 = y0(:);
    dx = dx(:);
    dy = dy(:);

    if ~(numel(x0) == numel(y0) ...
            && numel(x0) == numel(dx) ...
            && numel(x0) == numel(dy))
        error('线段起点和增量的长度不一致。');
    end

    n = numel(x0);

    line_x = nan(3,n);
    line_y = nan(3,n);

    line_x(1,:) = x0.';
    line_x(2,:) = (x0 + dx).';

    line_y(1,:) = y0.';
    line_y(2,:) = (y0 + dy).';

    line_x = line_x(:);
    line_y = line_y(:);
end
