if ~exist('odom_log','var') || ~exist('status_record','var')
    error(['缺少 odom_log 或 status_record。', newline, ...
           '请先运行 fusion_estimator_use.m，并保留工作区变量，不要 clear 工作区。']);
end

if ~exist('range','var') || isempty(range)
    range = 1:size(odom_log,1);
end

if ~exist('WordSize','var') || isempty(WordSize)
    WordSize = 16;
end

% ========================================================================
% 可调参数
% ========================================================================

% ---------------- 平移碰撞参数 ----------------
% 单位直接使用m/s^2，不再把检测阈值与绘图缩放混在一起。
acc_threshold = 2.5;             % 水平姿态、RobotTypePar=1时的基础阈值 /m/s^2

% 可在运行本脚本前于工作区手动设置RobotTypePar；
% 若没有设置，则默认使用1。
if ~exist('RobotTypePar','var') || isempty(RobotTypePar)
    RobotTypePar = 1;
end

% Roll与Pitch绝对值之和增大后，平移加速度阈值连续线性提高：
% tilt_factor = max((tilt_deg - tilt_start_deg)/tilt_step_deg + 1, 1)
tilt_start_deg = 10;
tilt_step_deg = 10;

collision_dead_time = 0.2;       % 同类碰撞事件的屏蔽时间 /s

velocity_average_frames = 10;    % 撞击前平均平移速度帧数，不含当前帧
velocity_min = 0.2;             % 平均平移速度最低值 /m/s

% abs(YawAcc)仅用于放宽平移碰撞中：
% “平移速度反方向与ImpactAcc方向”的允许夹角。
angle_base_deg = 30;
angle_yaw_gain_deg = 5;
angle_max_deg = 75;

% 撞击点方向距离正前、正右、正后、正左不超过该角度时，
% 判断为对应面的两腿同时碰撞。
face_snap_deg = 15;

% ---------------- 旋转碰撞参数 ----------------
rotation_yaw_velocity_average_frames = 30; % 撞击前平均Yaw角速度帧数
rotation_yaw_velocity_threshold = 0.8;     % |平均YawVel|阈值 /rad/s
rotation_yaw_acc_threshold = 12.0;          % |瞬时YawAcc|阈值 /rad/s^2

% 旋转碰撞独立使用的水平ImpactAcc阈值。
% 它不使用平移的acc_threshold_dynamic。
rotation_acc_threshold = 0.5;              % /m/s^2

% 实际ImpactAcc方向与候选腿“反向切向速度方向”之间的
% 最大允许夹角。数值越大越宽松，范围[0,90]度。
rotation_acc_direction_allow_deg = 80;

% ---------------- 绘图参数 ----------------
% 该比例只用于显示，不参与任何碰撞判断。
acc_plot_scale = 1/10;

yaw_acc_min = 0.00;
yaw_arc_gain = 0.15;
yaw_arc_max = 1.5*pi;
yaw_arc_radius = 0.25;
yaw_arc_point_num = 30;
vector_line_width = 1.4;

% ========================================================================
% 腿编号、碰撞编码和图例
% ========================================================================

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

leg_dir = leg_xy ./ vecnorm(leg_xy,2,2);

% ---------------- 旋转判腿方向自检 ----------------
% 坐标约定：
%   +X向前，+Y向左，+Yaw逆时针。
%
% 预期关系：
%   逆时针 + ImpactAcc[+X,-Y] -> Leg0
%   逆时针 + ImpactAcc[-X,+Y] -> Leg3
%   顺时针 + ImpactAcc[+X,+Y] -> Leg1
%   顺时针 + ImpactAcc[-X,-Y] -> Leg2
rotation_direction_self_check(leg_dir);

% collision_face_code：
%   0  = ALeg0
%   1  = ALeg1
%   2  = ALeg2
%   3  = ALeg3
%   10 = ALeg01，正前
%   13 = ALeg13，正右
%   32 = ALeg32，正后
%   20 = ALeg20，正左
%
% collision_event_record：
%   [原始帧号, 时间/s, collision_face_code]
%
% collision_event_detail_record：
%   [原始帧号, 时间/s, collision_face_code, collision_type]
%   collision_type：1=平移碰撞，2=旋转碰撞

leg_color = [
    1.00 0.00 1.00   % Leg0
    0.00 1.00 1.00   % Leg1
    0.85 0.65 0.00   % Leg2
    0.00 0.65 0.00   % Leg3
];

% 图例按照0 -> 1 -> 3 -> 2的机体周向顺序排列。
acc_category_code = [0,10,1,13,3,32,2,20];

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

% 单腿碰撞使用实线；正前后左右的双腿碰撞使用虚线。
acc_category_style = {'-','--','-','--','-','--','-','--'};

% ========================================================================
% 参数与数据检查
% ========================================================================

range = range(:);
range = range(isfinite(range));
range = range(range == fix(range));
range = range(range >= 1);
range = range(range <= size(odom_log,1));
range = range(range <= size(status_record,2));

if isempty(range)
    error('range中没有有效的数据序号。');
end

if any(diff(range) <= 0)
    error('range必须严格递增，不能包含重复或倒序索引。');
end

if size(odom_log,2) < 23
    error('odom_log列数不足，需要至少包含第23列机器狗质量。');
end

if size(status_record,1) < 31
    error(['status_record行数不足，无法读取四条腿的XY足端等效力。', newline, ...
           '请先完整运行 fusion_estimator_use.m。']);
end

validate_positive_scalar(acc_threshold,'acc_threshold');
validate_positive_scalar(RobotTypePar,'RobotTypePar');
validate_nonnegative_scalar(tilt_start_deg,'tilt_start_deg');
validate_positive_scalar(tilt_step_deg,'tilt_step_deg');
validate_nonnegative_scalar(collision_dead_time,'collision_dead_time');

validate_positive_integer(velocity_average_frames, ...
    'velocity_average_frames');

validate_nonnegative_scalar(velocity_min,'velocity_min');
validate_nonnegative_scalar(angle_base_deg,'angle_base_deg');
validate_nonnegative_scalar(angle_yaw_gain_deg,'angle_yaw_gain_deg');
validate_positive_scalar(angle_max_deg,'angle_max_deg');

if angle_base_deg > angle_max_deg
    error('angle_base_deg不能大于angle_max_deg。');
end

if face_snap_deg < 0 || face_snap_deg >= 45
    error('face_snap_deg必须位于[0,45)度。');
end

validate_positive_integer(rotation_yaw_velocity_average_frames, ...
    'rotation_yaw_velocity_average_frames');

validate_nonnegative_scalar(rotation_yaw_velocity_threshold, ...
    'rotation_yaw_velocity_threshold');

validate_nonnegative_scalar(rotation_yaw_acc_threshold, ...
    'rotation_yaw_acc_threshold');

validate_nonnegative_scalar(rotation_acc_threshold, ...
    'rotation_acc_threshold');

if rotation_acc_direction_allow_deg < 0 ...
        || rotation_acc_direction_allow_deg > 90 ...
        || ~isscalar(rotation_acc_direction_allow_deg) ...
        || ~isfinite(rotation_acc_direction_allow_deg)
    error('rotation_acc_direction_allow_deg必须位于[0,90]度。');
end

validate_positive_scalar(acc_plot_scale,'acc_plot_scale');
validate_nonnegative_scalar(yaw_acc_min,'yaw_acc_min');
validate_nonnegative_scalar(yaw_arc_gain,'yaw_arc_gain');
validate_positive_scalar(yaw_arc_max,'yaw_arc_max');
validate_positive_scalar(yaw_arc_radius,'yaw_arc_radius');
validate_positive_integer(yaw_arc_point_num,'yaw_arc_point_num');
validate_positive_scalar(vector_line_width,'vector_line_width');

sample_id = double(range);
time_sec = odom_log(range,1);

if any(~isfinite(time_sec))
    error('odom_log(:,1)中存在无效时间。');
end

if any(diff(time_sec) < 0)
    error('odom_log(:,1)时间不是单调递增。');
end

data_num = numel(range);

% ========================================================================
% 读取观测
% ========================================================================

roll = odom_log(range,11);
pitch = odom_log(range,12);
yaw = odom_log(range,13);
yaw_acc = odom_log(range,19);
dog_mass = odom_log(range,23);

imu_acc_x_world = odom_log(range,8);
imu_acc_y_world = odom_log(range,9);

% ========================================================================
% 计算外部冲击加速度ImpactAcc
% ========================================================================

% status 21~32中四条腿XY足端等效力：
% Leg0：[21,22]
% Leg1：[24,25]
% Leg2：[27,28]
% Leg3：[30,31]
force_sum_x_world = ...
    sum(status_record([49,52,55,58],range),1).';

force_sum_y_world = ...
    sum(status_record([50,53,56,59],range),1).';

mass_valid = isfinite(dog_mass) & dog_mass > 0;

facc_x_world = nan(data_num,1);
facc_y_world = nan(data_num,1);

facc_x_world(mass_valid) = ...
    force_sum_x_world(mass_valid) ./ dog_mass(mass_valid);

facc_y_world(mass_valid) = ...
    force_sum_y_world(mass_valid) ./ dog_mass(mass_valid);

% 当前足端力直接计算的FAcc方向与足端实际给身体的加速度反向：
% ImpactAcc = IMU Acc + FAcc。
impact_acc_x_world = imu_acc_x_world + facc_x_world;
impact_acc_y_world = imu_acc_y_world + facc_y_world;

impact_acc_valid = ...
    mass_valid ...
    & isfinite(impact_acc_x_world) ...
    & isfinite(impact_acc_y_world);

impact_acc_norm = hypot(impact_acc_x_world,impact_acc_y_world);

% ========================================================================
% 平移碰撞的动态加速度阈值
% ========================================================================

% odom_log(:,11:12)单位为rad。
tilt_abs_sum_deg = rad2deg(abs(roll) + abs(pitch));
attitude_valid = isfinite(tilt_abs_sum_deg);

tilt_threshold_factor = max( ...
    (tilt_abs_sum_deg - tilt_start_deg) / tilt_step_deg + 1, ...
    1);

% 单位保持为m/s^2。
acc_threshold_dynamic = ...
    acc_threshold * RobotTypePar .* tilt_threshold_factor;

% ========================================================================
% 撞击前历史平均速度
% ========================================================================

vel_mean_world = previous_window_mean( ...
    odom_log,range,[5,6],velocity_average_frames);

vel_mean_x_world = vel_mean_world(:,1);
vel_mean_y_world = vel_mean_world(:,2);
vel_mean_norm = hypot(vel_mean_x_world,vel_mean_y_world);

yaw_vel_mean = previous_window_mean( ...
    odom_log,range,16,rotation_yaw_velocity_average_frames);

% ========================================================================
% 只为判腿将ImpactAcc旋转到当前身体Yaw坐标系
% ========================================================================

cy = cos(yaw);
sy = sin(yaw);

impact_acc_x_body = ...
    cy .* impact_acc_x_world + sy .* impact_acc_y_world;

impact_acc_y_body = ...
   -sy .* impact_acc_x_world + cy .* impact_acc_y_world;

impact_acc_body_valid = ...
    isfinite(impact_acc_x_body) ...
    & isfinite(impact_acc_y_body);

% ========================================================================
% 原有平移碰撞辨识
% ========================================================================

reverse_vel_x_world = -vel_mean_x_world;
reverse_vel_y_world = -vel_mean_y_world;

translation_angle_error_rad = nan(data_num,1);

translation_angle_valid = ...
    isfinite(reverse_vel_x_world) ...
    & isfinite(reverse_vel_y_world) ...
    & impact_acc_valid ...
    & isfinite(yaw_acc) ...
    & vel_mean_norm >= velocity_min ...
    & impact_acc_norm > eps;

angle_cross = ...
    reverse_vel_x_world .* impact_acc_y_world ...
    - reverse_vel_y_world .* impact_acc_x_world;

angle_dot = ...
    reverse_vel_x_world .* impact_acc_x_world ...
    + reverse_vel_y_world .* impact_acc_y_world;

translation_angle_error_rad(translation_angle_valid) = atan2( ...
    abs(angle_cross(translation_angle_valid)), ...
    angle_dot(translation_angle_valid));

translation_angle_allow_deg = min( ...
    angle_base_deg ...
    + angle_yaw_gain_deg .* abs(yaw_acc), ...
    angle_max_deg);

translation_angle_allow_rad = ...
    deg2rad(translation_angle_allow_deg);

% 平移碰撞阈值直接比较m/s^2。
translation_acc_candidate = ...
    attitude_valid ...
    & impact_acc_valid ...
    & impact_acc_norm > acc_threshold_dynamic;

translation_candidate_raw = ...
    translation_acc_candidate ...
    & translation_angle_valid ...
    & translation_angle_error_rad ...
        <= translation_angle_allow_rad;

% 先判断能否确定碰撞腿/面，再应用碰撞死区。
translation_face_code_raw = -ones(data_num,1);
translation_raw_id = find(translation_candidate_raw);

for jj = 1:numel(translation_raw_id)
    ii = translation_raw_id(jj);

    if ~impact_acc_body_valid(ii)
        continue;
    end

    a_body = [
        impact_acc_x_body(ii), ...
        impact_acc_y_body(ii)
    ];

    a_norm = norm(a_body);

    if ~isfinite(a_norm) || a_norm <= eps
        continue;
    end

    contact_dir = -a_body / a_norm;

    translation_face_code_raw(ii) = classify_contact_face( ...
        contact_dir,leg_dir,face_snap_deg);
end

translation_classified_candidate = ...
    translation_candidate_raw ...
    & translation_face_code_raw >= 0;

translation_event_mask = apply_dead_time( ...
    translation_classified_candidate, ...
    time_sec, ...
    collision_dead_time);

translation_valid_collision_id = find(translation_event_mask);

translation_collision_face_code = -ones(data_num,1);

translation_collision_face_code(translation_valid_collision_id) = ...
    translation_face_code_raw(translation_valid_collision_id);

% ========================================================================
% 新增旋转碰撞辨识
% ========================================================================

% 旋转碰撞发生条件相互独立：
% 1. 撞击前平均Yaw角速度超过阈值；
% 2. 当前瞬时Yaw角加速度超过阈值；
% 3. 当前水平ImpactAcc超过独立的rotation_acc_threshold；
% 4. 平均Yaw角速度与瞬时Yaw角加速度方向相反。
%
% 第4项表示碰撞产生的角加速度正在阻碍当前旋转：
%   逆时针旋转：YawVelMean > 0，要求YawAcc < 0；
%   顺时针旋转：YawVelMean < 0，要求YawAcc > 0。
%
% 不使用平移的acc_threshold_dynamic；
% 不要求平移碰撞分支未命中。
rotation_direction_valid = ...
    isfinite(yaw_vel_mean) ...
    & isfinite(yaw_acc) ...
    & yaw_vel_mean .* yaw_acc < 0;

rotation_candidate_raw = ...
    rotation_direction_valid ...
    & impact_acc_valid ...
    & impact_acc_body_valid ...
    & abs(yaw_vel_mean) >= rotation_yaw_velocity_threshold ...
    & abs(yaw_acc) >= rotation_yaw_acc_threshold ...
    & impact_acc_norm >= rotation_acc_threshold;

rotation_face_code_raw = -ones(data_num,1);
rotation_leg_direction_angle_deg = nan(data_num,1);

rotation_raw_id = find(rotation_candidate_raw);

for jj = 1:numel(rotation_raw_id)
    ii = rotation_raw_id(jj);

    a_body = [
        impact_acc_x_body(ii), ...
        impact_acc_y_body(ii)
    ];

    a_norm = norm(a_body);

    if ~isfinite(a_norm) || a_norm <= eps
        continue;
    end

    [rotation_face_code_raw(ii),best_angle_deg] = ...
        classify_rotation_leg( ...
            a_body, ...
            yaw_vel_mean(ii), ...
            leg_dir, ...
            rotation_acc_direction_allow_deg);

    rotation_leg_direction_angle_deg(ii) = best_angle_deg;
end

% 同样先完成判腿，再应用旋转碰撞自身的死区。
rotation_classified_candidate = ...
    rotation_candidate_raw ...
    & rotation_face_code_raw >= 0;

rotation_event_mask = apply_dead_time( ...
    rotation_classified_candidate, ...
    time_sec, ...
    collision_dead_time);

rotation_valid_collision_id = find(rotation_event_mask);

rotation_collision_face_code = -ones(data_num,1);

rotation_collision_face_code(rotation_valid_collision_id) = ...
    rotation_face_code_raw(rotation_valid_collision_id);

% ========================================================================
% 合并平移与旋转碰撞
% ========================================================================

% 同一帧同时命中时保持平移碰撞优先。
collision_face_code = -ones(data_num,1);
collision_type = zeros(data_num,1);

collision_face_code(translation_valid_collision_id) = ...
    translation_collision_face_code(translation_valid_collision_id);

collision_type(translation_valid_collision_id) = 1;

rotation_only_id = rotation_valid_collision_id( ...
    collision_type(rotation_valid_collision_id) == 0);

collision_face_code(rotation_only_id) = ...
    rotation_collision_face_code(rotation_only_id);

collision_type(rotation_only_id) = 2;

valid_collision_id = find(collision_type > 0);

collision_event_record = [
    sample_id(valid_collision_id), ...
    time_sec(valid_collision_id), ...
    collision_face_code(valid_collision_id)
];

collision_event_detail_record = [
    collision_event_record, ...
    collision_type(valid_collision_id)
];

draw_local_id = valid_collision_id;
translation_draw_local_id = translation_valid_collision_id;

% ========================================================================
% 绘图
% ========================================================================

fig = figure(2);
clf(fig);

ax = axes('Parent',fig);
hold(ax,'on');
grid(ax,'on');

h_x = plot(ax,sample_id,odom_log(range,16)/2, ...
    'b','LineWidth',1.2);

h_y = plot(ax,sample_id,odom_log(range,19)/20, ...
    'b-.','LineWidth',1.2);

h_vel = plot(ax,nan,nan,'r-.', ...
    'LineWidth',vector_line_width);

h_yaw = plot(ax,nan,nan,'k-', ...
    'LineWidth',vector_line_width);

h_acc_category = gobjects(numel(acc_category_code),1);

for category_id = 1:numel(acc_category_code)
    h_acc_category(category_id) = plot(ax,nan,nan, ...
        acc_category_style{category_id}, ...
        'Color',acc_category_color(category_id,:), ...
        'LineWidth',vector_line_width);
end

yline(ax,0, ...
    'Color',[0.5 0.5 0.5], ...
    'LineStyle',':', ...
    'HandleVisibility','off');

x_margin = max(1,0.01*(sample_id(end)-sample_id(1)+1));

xlim(ax,[sample_id(1)-x_margin,sample_id(end)+x_margin]);
ylim(ax,[-1.5,1.5]);

xlabel(ax,'Sample Index','FontSize',WordSize);
ylabel(ax,'Position / Collision Direction','FontSize',WordSize);

title(ax, ...
    'Collision Detection: Translation Lines / Rotation Arrows', ...
    'FontSize',WordSize);

drawnow;

% ---------------- 屏幕比例补偿 ----------------
% 世界系向量显示约定：
%   向上 = 世界+X
%   向左 = 世界+Y
%   逆时针 = +YawAcc
old_units = ax.Units;
ax.Units = 'pixels';
ax_pos = ax.Position;
ax.Units = old_units;

x_range = diff(xlim(ax));
y_range = diff(ylim(ax));

if ax_pos(3) <= 0 ...
        || ax_pos(4) <= 0 ...
        || x_range <= 0 ...
        || y_range <= 0
    error('坐标轴尺寸或显示范围无效。');
end

x_data_per_y_visual = ...
    x_range * ax_pos(4) / (y_range * ax_pos(3));

if ~isfinite(x_data_per_y_visual) ...
        || x_data_per_y_visual <= 0
    error('坐标轴显示比例计算失败。');
end

% ---------------- 平移碰撞的平均速度线段 ----------------
if ~isempty(translation_draw_local_id)
    vel_up = vel_mean_x_world(translation_draw_local_id);
    vel_left = vel_mean_y_world(translation_draw_local_id);

    vel_dx = -vel_left * x_data_per_y_visual;
    vel_dy = vel_up;

    [vel_line_x,vel_line_y] = make_line_segments( ...
        sample_id(translation_draw_local_id), ...
        zeros(size(translation_draw_local_id)), ...
        vel_dx,vel_dy);

    plot(ax,vel_line_x,vel_line_y,'r-.', ...
        'LineWidth',vector_line_width, ...
        'HandleVisibility','off');
end

% ---------------- ImpactAcc分类绘制 ----------------
% 平移碰撞：普通线段。
% 旋转碰撞：带箭头向量。
for category_id = 1:numel(acc_category_code)
    code_now = acc_category_code(category_id);

    category_collision_id = draw_local_id( ...
        collision_face_code(draw_local_id) == code_now);

    if isempty(category_collision_id)
        continue;
    end

    translation_category_id = category_collision_id( ...
        collision_type(category_collision_id) == 1);

    rotation_category_id = category_collision_id( ...
        collision_type(category_collision_id) == 2);

    if ~isempty(translation_category_id)
        acc_up = ...
            impact_acc_x_world(translation_category_id) ...
            * acc_plot_scale;

        acc_left = ...
            impact_acc_y_world(translation_category_id) ...
            * acc_plot_scale;

        acc_dx = -acc_left * x_data_per_y_visual;
        acc_dy = acc_up;

        [acc_line_x,acc_line_y] = make_line_segments( ...
            sample_id(translation_category_id), ...
            zeros(size(translation_category_id)), ...
            acc_dx,acc_dy);

        plot(ax,acc_line_x,acc_line_y, ...
            acc_category_style{category_id}, ...
            'Color',acc_category_color(category_id,:), ...
            'LineWidth',vector_line_width, ...
            'HandleVisibility','off');
    end

    if ~isempty(rotation_category_id)
        acc_up = ...
            impact_acc_x_world(rotation_category_id) ...
            * acc_plot_scale;

        acc_left = ...
            impact_acc_y_world(rotation_category_id) ...
            * acc_plot_scale;

        acc_dx = -acc_left * x_data_per_y_visual;
        acc_dy = acc_up;

        [arrow_x,arrow_y] = make_arrow_segments( ...
            sample_id(rotation_category_id), ...
            zeros(size(rotation_category_id)), ...
            acc_dx,acc_dy, ...
            x_data_per_y_visual);

        plot(ax,arrow_x,arrow_y, ...
            acc_category_style{category_id}, ...
            'Color',acc_category_color(category_id,:), ...
            'LineWidth',vector_line_width, ...
            'HandleVisibility','off');
    end
end

% ---------------- Yaw角加速度圆弧 ----------------
arc_local_id = draw_local_id( ...
    isfinite(yaw_acc(draw_local_id)) ...
    & abs(yaw_acc(draw_local_id)) > yaw_acc_min);

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
        'LineWidth',vector_line_width, ...
        'HandleVisibility','off');
end

legend( ...
    [h_x,h_y,h_vel,h_yaw,h_acc_category.'], ...
    [{'X/2','Y/20','Vel','aYaw'},acc_category_name], ...
    'FontSize',WordSize);

% ========================================================================
% 检测结果输出
% ========================================================================

fprintf( ...
    ['[Collision Plot] translation=%d, rotation=%d, total=%d\n'], ...
    numel(translation_valid_collision_id), ...
    numel(rotation_only_id), ...
    size(collision_event_record,1));

fprintf( ...
    ['[Translation Parameters] AccThreshold=%.3f m/s^2, ', ...
     'RobotTypePar=%.3f, VelFrames=%d\n'], ...
    acc_threshold, ...
    RobotTypePar, ...
    velocity_average_frames);

fprintf( ...
    ['[Rotation Parameters] ImpactAccThreshold=%.3f m/s^2, ', ...
     'YawVelFrames=%d, YawVelThreshold=%.3f rad/s, ', ...
     'YawAccThreshold=%.3f rad/s^2, DirectionAllow=%.1f deg, ', ...
     'YawDirectionConstraint=opposite\n'], ...
    rotation_acc_threshold, ...
    rotation_yaw_velocity_average_frames, ...
    rotation_yaw_velocity_threshold, ...
    rotation_yaw_acc_threshold, ...
    rotation_acc_direction_allow_deg);

fprintf(['[Collision Record] collision_event_record = ', ...
    '[frame, time_s, face_code]\n']);

fprintf(['[Collision Detail] collision_event_detail_record = ', ...
    '[frame, time_s, face_code, type], ', ...
    'type:1=translation,2=rotation\n']);

for jj = 1:size(collision_event_detail_record,1)
    ii = valid_collision_id(jj);
    type_now = collision_event_detail_record(jj,4);

    if type_now == 1
        fprintf( ...
            ['[Collision] frame=%d t=%.3f type=translation ', ...
             'code=%d ImpactAcc=%.3f m/s^2 ', ...
             'AccThreshold=%.3f m/s^2 ', ...
             'angle=%.1f deg allow=%.1f deg ', ...
             '|YawAcc|=%.3f rad/s^2\n'], ...
            collision_event_detail_record(jj,1), ...
            collision_event_detail_record(jj,2), ...
            collision_event_detail_record(jj,3), ...
            impact_acc_norm(ii), ...
            acc_threshold_dynamic(ii), ...
            rad2deg(translation_angle_error_rad(ii)), ...
            translation_angle_allow_deg(ii), ...
            abs(yaw_acc(ii)));
    else
        fprintf( ...
            ['[Collision] frame=%d t=%.3f type=rotation ', ...
             'code=%d ImpactAcc=%.3f m/s^2 ', ...
             'ImpactAccThreshold=%.3f m/s^2 ', ...
             'YawVelMean=%.3f rad/s ', ...
             'YawAcc=%.3f rad/s^2 ', ...
             'YawVel*YawAcc=%.3f ', ...
             'legAngle=%.1f deg allow=%.1f deg\n'], ...
            collision_event_detail_record(jj,1), ...
            collision_event_detail_record(jj,2), ...
            collision_event_detail_record(jj,3), ...
            impact_acc_norm(ii), ...
            rotation_acc_threshold, ...
            yaw_vel_mean(ii), ...
            yaw_acc(ii), ...
            yaw_vel_mean(ii)*yaw_acc(ii), ...
            rotation_leg_direction_angle_deg(ii), ...
            rotation_acc_direction_allow_deg);
    end
end


function mean_value = previous_window_mean( ...
    data,range,column_id,window_size)

    range = range(:);
    column_id = column_id(:).';

    mean_value = nan(numel(range),numel(column_id));

    for ii = 1:numel(range)
        first_id = range(ii) - window_size;
        last_id = range(ii) - 1;

        if first_id < 1
            continue;
        end

        window_data = data(first_id:last_id,column_id);

        if any(~isfinite(window_data(:)))
            continue;
        end

        mean_value(ii,:) = mean(window_data,1);
    end

    if numel(column_id) == 1
        mean_value = mean_value(:,1);
    end
end


function event_mask = apply_dead_time( ...
    candidate_mask,time_sec,dead_time)

    candidate_mask = logical(candidate_mask(:));
    time_sec = time_sec(:);

    if numel(candidate_mask) ~= numel(time_sec)
        error('候选事件和时间数组长度不一致。');
    end

    event_mask = false(size(candidate_mask));
    last_event_time = -inf;

    for ii = 1:numel(candidate_mask)
        if candidate_mask(ii) ...
                && time_sec(ii) - last_event_time >= dead_time
            event_mask(ii) = true;
            last_event_time = time_sec(ii);
        end
    end
end


function rotation_direction_self_check(leg_dir)

    test_yaw_vel = [1;1;-1;-1];

    test_impact_acc = [
         1,-1
        -1, 1
         1, 1
        -1,-1
    ];

    expected_leg = [0;3;1;2];

    for test_id = 1:4
        [leg_id,~] = classify_rotation_leg( ...
            test_impact_acc(test_id,:), ...
            test_yaw_vel(test_id), ...
            leg_dir, ...
            90);

        if leg_id ~= expected_leg(test_id)
            error(['旋转碰撞方向自检失败：测试%d期望Leg%d，', ...
                   '实际得到Leg%d。'], ...
                test_id,expected_leg(test_id),leg_id);
        end
    end
end


function [leg_id,best_angle_deg] = classify_rotation_leg( ...
    impact_acc_body,yaw_velocity,leg_dir,allow_deg)

    leg_id = -1;
    best_angle_deg = nan;

    impact_acc_body = impact_acc_body(:).';
    impact_norm = norm(impact_acc_body);

    if numel(impact_acc_body) ~= 2 ...
            || any(~isfinite(impact_acc_body)) ...
            || ~isfinite(impact_norm) ...
            || impact_norm <= eps ...
            || ~isscalar(yaw_velocity) ...
            || ~isfinite(yaw_velocity) ...
            || yaw_velocity == 0
        return;
    end

    impact_acc_dir = impact_acc_body / impact_norm;

    if yaw_velocity > 0
        candidate_leg_id = [0,3];
    else
        candidate_leg_id = [1,2];
    end

    % 候选腿相对机体中心的径向单位方向。
    candidate_r_dir = leg_dir(candidate_leg_id + 1,:);

    % 对二维Yaw旋转：
    %   v_tangent = YawVel * [-r_y, r_x]
    %
    % 碰撞产生的ImpactAcc应大致与碰撞前腿部切向速度反向。
    rotation_sign = sign(yaw_velocity);

    candidate_tangent_velocity_dir = ...
        rotation_sign * [
            -candidate_r_dir(:,2), ...
             candidate_r_dir(:,1)
        ];

    expected_impact_acc_dir = ...
        -candidate_tangent_velocity_dir;

    candidate_score = ...
        expected_impact_acc_dir * impact_acc_dir.';

    candidate_score = max(min(candidate_score,1),-1);
    candidate_angle_deg = acosd(candidate_score);

    [best_angle_deg,best_candidate_index] = ...
        min(candidate_angle_deg);

    if best_angle_deg <= allow_deg
        leg_id = candidate_leg_id(best_candidate_index);
    end
end


function face_code = classify_contact_face( ...
    contact_dir,leg_dir,face_snap_deg)

    face_code = -1;

    if numel(contact_dir) ~= 2 ...
            || any(~isfinite(contact_dir))
        return;
    end

    contact_dir = contact_dir(:).';
    contact_norm = norm(contact_dir);

    if ~isfinite(contact_norm) || contact_norm <= eps
        return;
    end

    contact_dir = contact_dir / contact_norm;
    contact_angle_deg = atan2d(contact_dir(2),contact_dir(1));

    if abs(wrap_angle_deg(contact_angle_deg - 0)) <= face_snap_deg
        face_code = 10;
    elseif abs(wrap_angle_deg(contact_angle_deg + 90)) <= face_snap_deg
        face_code = 13;
    elseif abs(wrap_angle_deg(contact_angle_deg - 180)) <= face_snap_deg
        face_code = 32;
    elseif abs(wrap_angle_deg(contact_angle_deg - 90)) <= face_snap_deg
        face_code = 20;
    else
        leg_score = leg_dir * contact_dir.';
        [~,best_leg_index] = max(leg_score);
        face_code = best_leg_index - 1;
    end
end


function angle_deg = wrap_angle_deg(angle_deg)

    angle_deg = mod(angle_deg + 180,360) - 180;
end


function [arrow_x,arrow_y] = make_arrow_segments( ...
    x0,y0,dx,dy,x_data_per_y_visual)

    x0 = x0(:);
    y0 = y0(:);
    dx = dx(:);
    dy = dy(:);

    if ~(numel(x0) == numel(y0) ...
            && numel(x0) == numel(dx) ...
            && numel(x0) == numel(dy))
        error('箭头起点和增量的长度不一致。');
    end

    if ~isscalar(x_data_per_y_visual) ...
            || ~isfinite(x_data_per_y_visual) ...
            || x_data_per_y_visual <= 0
        error('箭头屏幕比例参数无效。');
    end

    n = numel(x0);

    visual_dx = dx / x_data_per_y_visual;
    visual_dy = dy;
    visual_len = hypot(visual_dx,visual_dy);

    valid = ...
        isfinite(x0) ...
        & isfinite(y0) ...
        & isfinite(dx) ...
        & isfinite(dy) ...
        & visual_len > 0;

    unit_x = zeros(n,1);
    unit_y = zeros(n,1);

    unit_x(valid) = visual_dx(valid) ./ visual_len(valid);
    unit_y(valid) = visual_dy(valid) ./ visual_len(valid);

    head_len = min( ...
        0.10*ones(n,1), ...
        0.35*visual_len);

    head_half_width = 0.55*head_len;

    tip_x = x0 + dx;
    tip_y = y0 + dy;

    back_x = ...
        tip_x - head_len .* unit_x * x_data_per_y_visual;

    back_y = ...
        tip_y - head_len .* unit_y;

    perp_x = -unit_y;
    perp_y = unit_x;

    wing1_x = ...
        back_x ...
        + head_half_width .* perp_x * x_data_per_y_visual;

    wing1_y = ...
        back_y + head_half_width .* perp_y;

    wing2_x = ...
        back_x ...
        - head_half_width .* perp_x * x_data_per_y_visual;

    wing2_y = ...
        back_y - head_half_width .* perp_y;

    arrow_x = nan(9,n);
    arrow_y = nan(9,n);

    arrow_x(1,:) = x0.';
    arrow_x(2,:) = tip_x.';
    arrow_y(1,:) = y0.';
    arrow_y(2,:) = tip_y.';

    arrow_x(4,:) = tip_x.';
    arrow_x(5,:) = wing1_x.';
    arrow_y(4,:) = tip_y.';
    arrow_y(5,:) = wing1_y.';

    arrow_x(7,:) = tip_x.';
    arrow_x(8,:) = wing2_x.';
    arrow_y(7,:) = tip_y.';
    arrow_y(8,:) = wing2_y.';

    arrow_x(:,~valid) = nan;
    arrow_y(:,~valid) = nan;

    arrow_x = arrow_x(:);
    arrow_y = arrow_y(:);
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


function validate_positive_scalar(value,name)

    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('%s必须是大于0的有限标量。',name);
    end
end


function validate_nonnegative_scalar(value,name)

    if ~isscalar(value) || ~isfinite(value) || value < 0
        error('%s必须是非负有限标量。',name);
    end
end


function validate_positive_integer(value,name)

    if ~isscalar(value) ...
            || ~isfinite(value) ...
            || value < 1 ...
            || value ~= fix(value)
        error('%s必须是正整数。',name);
    end
end
