clear all;clc;
cd(fileparts(mfilename('fullpath')));

% CSV_PATH = 'Data/MW_D_GCTest2'; DogMode = 140;
% CSV_PATH = 'Data/MW_Vibration_OnRack_Pose'; DogMode = 140;
CSV_PATH = 'Data/MW_D_XY150Z10'; DogMode = 140;
% CSV_PATH = 'Data/MW_D_Car'; DogMode = 140;
% CSV_PATH = 'Data/MP_XY150Z10'; DogMode = 141;
% CSV_PATH = 'Data/MW_XY150Z10'; DogMode = 142;
% CSV_PATH = 'Data/LW_Collision_Wheel'; DogMode = 160;
% CSV_PATH = 'Data/LW_Collision_Trot'; DogMode = 160;

used_lines = 300000;

data = readmatrix(CSV_PATH);
N = size(data,1);

fid = fopen(CSV_PATH,'r');
C = textscan(fid,'%s%*[^\n]',N+1,'Delimiter',',');
fclose(fid);

time_strs = string(C{1}(2:end));
pat = '\.(\d{1,3})(?!\d)';
time_strs_fix = regexprep(time_strs,pat,'.${pad($1,3,"left","0")}');
t0 = datetime(time_strs_fix(1),'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
t = datetime(time_strs_fix,'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
ts_ms_all = int64(round(milliseconds(t-t0)));
data(:,1) = ts_ms_all;

N = min(size(data,1),used_lines);
data = data(1:N,:);
ts_ms_all = ts_ms_all(1:N);

base0 = 2;
if length(data(1,:)) == 220
    stride = 13;
elseif length(data(1,:)) == 236
    stride = 14;
elseif length(data(1,:)) > 236
    stride = 14;
    data = data(:,1:236);
else
    error('Unsupported CSV column count: %d',length(data(1,:)));
end

motor_num = 16;
imu0 = base0 + motor_num*stride;

q16 = data(:,base0+(0:15)*stride+6);
dq16 = data(:,base0+(0:15)*stride+7);
tau16 = data(:,base0+(0:15)*stride+8);

acc = data(:,imu0+(1:3));
gyro = data(:,imu0+(4:6));
quat = data(:,imu0+(7:10));

fusion_estimator_mex('reset');

status_ = zeros(100,1,'double');
status_(1) = DogMode;
status_ = fusion_estimator_mex('status',status_);

status_(1) = 2;
status_ = fusion_estimator_mex('status',status_);

status_(1:7) = 1;
status_ = fusion_estimator_mex('status',status_);

Time_log = nan(N,1);
log_PositionXYZ = nan(N,9,'single');
log_OrientationRPY = nan(N,9,'single');
log_FootfallAverage = nan(N,3,'single');
log_FootLandedProbability = nan(N,4,'single');
log_DogWeight = nan(N,1,'single');
log_LegCollisionDetect = zeros(N,1,'double');
log_JointsBodyWFPosition = nan(3,4,4,N,'single');
log_JointsBodyWFEffort = nan(3,4,4,N,'single');
log_MotorGravityCompensate = nan(3,4,N,'single');

range = 1:N;

tic
for k = range
    % if k == 10
    %     status_(1) = 3;
    %     status_ = fusion_estimator_mex('status',status_);
    % end

    ts_ms = ts_ms_all(k);
    out = fusion_estimator_mex('step',ts_ms,q16(k,:),dq16(k,:),tau16(k,:),acc(k,:),gyro(k,:),quat(k,:));

    Time_log(k) = double(ts_ms)/1000;
    log_PositionXYZ(k,:) = out.PositionXYZ(:).';
    log_OrientationRPY(k,:) = out.OrientationRPY(:).';
    log_FootfallAverage(k,:) = out.FootfallAverage(:).';
    log_FootLandedProbability(k,:) = out.FootLandedProbability(:).';
    log_DogWeight(k) = out.DogWeight;
    log_LegCollisionDetect(k) = double(out.LegCollisionDetect);
    log_JointsBodyWFPosition(:,:,:,k) = out.JointsBodyWFPosition;
    log_JointsBodyWFEffort(:,:,:,k) = out.JointsBodyWFEffort;
    log_MotorGravityCompensate(:,:,k) = out.MotorGravityCompensate;

    % if mod(k,1000) == 0
    %     fprintf('[%d] t=%.1f x=%.1f y=%.1f z=%.1f r=%.2f p=%.2f y=%.2f\n',k,Time_log(k),log_PositionXYZ(k,1),log_PositionXYZ(k,4),log_PositionXYZ(k,7),log_OrientationRPY(k,1),log_OrientationRPY(k,4),log_OrientationRPY(k,7));
    % end
end
toc

fusion_estimator_mex('reset');
clear fusion_estimator_mex
fprintf('[DONE] frames=%d\n',k);

ZoomTime = [0,Time_log(end)];
% ZoomTime = [0,25];

WordSize = 25;

PlotFusionEstimator = @plot_fusion_estimator_logs_local;
PlotDogMotion = @plot_dog_motion_local;

PlotFusionEstimator()
% PlotDogMotion()
















































function plot_fusion_estimator_logs_local()

    required_variables = {'Time_log','log_PositionXYZ','log_OrientationRPY','log_FootLandedProbability','log_JointsBodyWFPosition','log_JointsBodyWFEffort','log_LegCollisionDetect','log_MotorGravityCompensate','tau16','dq16','range'};
    
    for i = 1:numel(required_variables)
        if ~evalin('base',['exist(''',required_variables{i},''',''var'')'])
            error('缺少工作区变量：%s。请先完整运行主脚本。',required_variables{i});
        end
    end

    Time_log = evalin('base','Time_log');
    log_PositionXYZ = evalin('base','log_PositionXYZ');
    log_OrientationRPY = evalin('base','log_OrientationRPY');
    log_FootLandedProbability = evalin('base','log_FootLandedProbability');
    log_JointsBodyWFPosition = evalin('base','log_JointsBodyWFPosition');
    log_LegCollisionDetect = evalin('base','log_LegCollisionDetect');
    log_JointsBodyWFEffort = evalin('base','log_JointsBodyWFEffort');
    log_MotorGravityCompensate = evalin('base','log_MotorGravityCompensate');
    tau16 = evalin('base','tau16');
    dq16 = evalin('base','dq16');
    range = evalin('base','range');

    if evalin('base','exist(''ZoomTime'',''var'')')
        ZoomTime = evalin('base','ZoomTime');
    else
        ZoomTime = [Time_log(1),Time_log(end)];
    end

    if evalin('base','exist(''WordSize'',''var'')')
        WordSize = evalin('base','WordSize');
    else
        WordSize = 16;
    end

    range = range(:);
    range = range(isfinite(range) & range == fix(range) & range >= 1 & range <= size(log_PositionXYZ,1));

    if isempty(range)
        error('range中没有有效帧。');
    end

    leg_names = {'FL','FR','RL','RR'};
    xyz_names = {'X','Y','Z'};
    node_names = {'hip','thigh','calf','foot'};
    t_plot = Time_log(range);

    figure(1);
    clf;

    tabs_state = uitabgroup;

    tab_pos = uitab(tabs_state,'Title','Position');
    tab_ori = uitab(tabs_state,'Title','Orientation');
    tab_vel_acc = uitab(tabs_state,'Title','Vel_Acc');
    tab_ang = uitab(tabs_state,'Title','AngVel_Acc');

    tableg0 = uitab(tabs_state,'Title','FL');
    tableg1 = uitab(tabs_state,'Title','FR');
    tableg2 = uitab(tabs_state,'Title','RL');
    tableg3 = uitab(tabs_state,'Title','RR');

    tab_gravity0 = uitab(tabs_state,'Title','FLGC');
    tab_gravity1 = uitab(tabs_state,'Title','FRGC');
    tab_gravity2 = uitab(tabs_state,'Title','RLGC');
    tab_gravity3 = uitab(tabs_state,'Title','RRGC');
    tab_distribution = uitab(tabs_state,'Title','Dist');

    subplot(1,1,1,'Parent',tab_pos);
    hold on;
    grid on;
    plot(t_plot,log_PositionXYZ(range,1),'r');
    plot(t_plot,log_PositionXYZ(range,4),'g');
    plot(t_plot,log_PositionXYZ(range,7),'b');
    plot(t_plot,log_LegCollisionDetect(range)/10,'k');
    xlim(ZoomTime);
    legend('X','Y','Z','CD','FontSize',WordSize);
    xlabel('Time /s','FontSize',WordSize);
    ylabel('Position /m','FontSize',WordSize);
    title('Position and Collision Detect','FontSize',WordSize);

    subplot(1,1,1,'Parent',tab_ori);
    hold on;
    grid on;
    plot(t_plot,log_OrientationRPY(range,1),'r');
    plot(t_plot,log_OrientationRPY(range,4),'g');
    plot(t_plot,log_OrientationRPY(range,7),'b');
    xlim(ZoomTime);
    legend('Roll','Pitch','Yaw','Location','best','FontSize',WordSize);
    xlabel('Time /s','FontSize',WordSize);
    ylabel('Angle /rad','FontSize',WordSize);
    title('Orientation','FontSize',WordSize);

    subplot(1,1,1,'Parent',tab_vel_acc);
    hold on;
    grid on;
    plot(t_plot,log_PositionXYZ(range,2),'r-');
    plot(t_plot,log_PositionXYZ(range,5),'g-');
    plot(t_plot,log_PositionXYZ(range,8),'b-');
    plot(t_plot,log_PositionXYZ(range,3),'--','Color',[1,0.3,0.3]);
    plot(t_plot,log_PositionXYZ(range,6),'--','Color',[0.3,1,0.3]);
    plot(t_plot,log_PositionXYZ(range,9),'--','Color',[0.3,0.3,1]);
    xlim(ZoomTime);
    legend('vX','vY','vZ','aX','aY','aZ','Location','best','FontSize',WordSize);
    xlabel('Time /s','FontSize',WordSize);
    ylabel('Velocity / Acceleration','FontSize',WordSize);
    title('Velocity and Acceleration','FontSize',WordSize);

    subplot(1,1,1,'Parent',tab_ang);
    hold on;
    grid on;
    plot(t_plot,log_OrientationRPY(range,2),'r-');
    plot(t_plot,log_OrientationRPY(range,5),'g-');
    plot(t_plot,log_OrientationRPY(range,8),'b-');
    plot(t_plot,log_OrientationRPY(range,3)/5,'r--');
    plot(t_plot,log_OrientationRPY(range,6)/5,'g--');
    plot(t_plot,log_OrientationRPY(range,9)/5,'b--');
    xlim(ZoomTime);
    legend('RollVel','PitchVel','YawVel','RollAcc','PitchAcc','YawAcc','Location','best','FontSize',WordSize);
    xlabel('Time /s','FontSize',WordSize);
    ylabel('Angular State','FontSize',WordSize);
    title('Angular Velocity and Acceleration','FontSize',WordSize);

    tab_list = [tableg0,tableg1,tableg2,tableg3];

    for leg = 0:3
        contact_mask = log_FootLandedProbability(range,leg+1) ~= -0.7;

        for xyz = 0:2
            subplot(3,1,xyz+1,'Parent',tab_list(leg+1));
            hold on;
            grid on;
            ylim([-1,1]);

            for node = 0:3
                node_position = reshape(log_JointsBodyWFPosition(xyz+1,node+1,leg+1,range),[],1);
                plot(t_plot,double(node_position).*contact_mask);
            end

            foot_effort = reshape(log_JointsBodyWFEffort(xyz+1,4,leg+1,range),[],1);
            h = plot(t_plot,double(foot_effort)/100.*contact_mask);
            h.Color(4) = 1.0;

            xlim(ZoomTime);
            ylabel(xyz_names{xyz+1});

            if xyz == 0
                title([leg_names{leg+1},' leg nodes position and force/100']);
                legend(node_names{:},'force/100','Location','northeast');
            end

            if xyz == 2
                xlabel('t / s');
            end
        end
    end

    distribution_index = {
        [1,5,9,13]
        [2,6,10,14]
        [3,7,11,15]
        [4,8,12,16]
    };

    distribution_color = {'r','g','b','k'};
    distribution_bin_num = 400;
    distribution_smooth_span = 7;

    dq_distribution_data = cell(4,1);
    tau_distribution_data = cell(4,1);

    for group_i = 1:4
        dq_distribution_data{group_i} = double(dq16(range,distribution_index{group_i}));
        tau_distribution_data{group_i} = double(tau16(range,distribution_index{group_i}));
        dq_distribution_data{group_i} = dq_distribution_data{group_i}(isfinite(dq_distribution_data{group_i}));
        tau_distribution_data{group_i} = tau_distribution_data{group_i}(isfinite(tau_distribution_data{group_i}));
    end

    dq_distribution_all = vertcat(dq_distribution_data{:});
    tau_distribution_all = vertcat(tau_distribution_data{:});

    dq_distribution_edge = linspace(min(dq_distribution_all),max(dq_distribution_all),distribution_bin_num+1);
    tau_distribution_edge = linspace(min(tau_distribution_all),max(tau_distribution_all),distribution_bin_num+1);

    dq_distribution_center = (dq_distribution_edge(1:end-1)+dq_distribution_edge(2:end))/2;
    tau_distribution_center = (tau_distribution_edge(1:end-1)+tau_distribution_edge(2:end))/2;

    subplot(1,1,1,'Parent',tab_distribution);
    hold on;
    grid on;

    for group_i = 1:4
        plot(dq_distribution_center,smoothdata(histcounts(dq_distribution_data{group_i},dq_distribution_edge,'Normalization','probability'),'gaussian',distribution_smooth_span),'--','Color',distribution_color{group_i});
        plot(tau_distribution_center,smoothdata(histcounts(tau_distribution_data{group_i},tau_distribution_edge,'Normalization','probability'),'gaussian',distribution_smooth_span),'-','Color',distribution_color{group_i});
    end

    xlabel('Value (dq: rad/s, tau: N·m)','FontSize',WordSize);
    ylabel('Proportion','FontSize',WordSize);
    title('dq16 and tau16 Distribution','FontSize',WordSize);
    legend('Hip dq','Hip tau','Thigh dq','Thigh tau','Calf dq','Calf tau','Foot dq','Foot tau','Location','best','FontSize',WordSize);

end


function plot_dog_motion_local()

    required_variables = {'Time_log','log_PositionXYZ','log_OrientationRPY','log_LegCollisionDetect','log_JointsBodyWFPosition','log_JointsBodyWFEffort','range'};

    for i = 1:numel(required_variables)
        if ~evalin('base',['exist(''',required_variables{i},''',''var'')'])
            error('缺少工作区变量：%s。请先完整运行主脚本。',required_variables{i});
        end
    end

    Time_log = evalin('base','Time_log');
    log_PositionXYZ = evalin('base','log_PositionXYZ');
    log_OrientationRPY = evalin('base','log_OrientationRPY');
    log_LegCollisionDetect = evalin('base','log_LegCollisionDetect');
    log_JointsBodyWFPosition = evalin('base','log_JointsBodyWFPosition');
    log_JointsBodyWFEffort = evalin('base','log_JointsBodyWFEffort');
    range = evalin('base','range');

    if evalin('base','exist(''WordSize'',''var'')')
        WordSize = evalin('base','WordSize');
    else
        WordSize = 16;
    end

    follow_init_len = 2.0;
    follow_init_z_down = 0.5;
    follow_init_z_up = 1.2;
    follow_init_az = 45;
    follow_init_el = 25;

    collision_show_duration = 1.0;
    collision_sphere_radius = 0.3;

    link_color = [0.00,0.75,0.00];
    joint_color = [0.00,0.00,1.00];
    joint_marker_size = 5;

    collision_color = [
        1.00,0.20,0.20
        1.00,0.00,1.00
        0.80,0.20,1.00
        0.85,0.65,0.00
        1.00,0.50,0.00
        0.00,0.65,0.00
        0.00,0.75,0.75
        0.00,1.00,1.00
    ];

    range = range(:).';
    range = range(isfinite(range) & range == fix(range) & range >= 1 & range <= size(log_PositionXYZ,1));

    if isempty(range)
        error('range中没有有效帧。');
    end

    vis_step = max(1,floor(numel(range)/3000));
    vis_idx = range(1:vis_step:end);

    valid_idx = all(isfinite(log_PositionXYZ(vis_idx,[1,4,7])),2) & isfinite(Time_log(vis_idx)) & isfinite(log_OrientationRPY(vis_idx,7));
    vis_idx = vis_idx(valid_idx);

    M = numel(vis_idx);

    if M < 1
        error('没有有效帧可用于三维运动显示。');
    end

    body_xyz = double(log_PositionXYZ(vis_idx,[1,4,7]));
    t_vis = double(Time_log(vis_idx));
    yaw_vis = double(log_OrientationRPY(vis_idx,7));

    node_w = nan(M,4,4,3);
    force_w = nan(M,4,3,3);

    for ii = 1:M
        k = vis_idx(ii);

        for leg = 1:4
            node_relative = double(squeeze(log_JointsBodyWFPosition(:,:,leg,k))).';
            node_w(ii,leg,:,:) = reshape(node_relative+body_xyz(ii,:),[1,1,4,3]);

            force_relative = double(squeeze(log_JointsBodyWFEffort(:,2:4,leg,k))).';
            force_w(ii,leg,:,:) = reshape(force_relative/500,[1,1,3,3]);
        end
    end

    collision_code = double(log_LegCollisionDetect(range));
    collision_valid = isfinite(collision_code) & collision_code >= 1 & collision_code <= 8 & collision_code == fix(collision_code);
    collision_frame = range(collision_valid);
    collision_events = [collision_frame(:),double(Time_log(collision_frame)),collision_code(collision_valid)];

    fig = figure(3);
    clf(fig);

    ax = axes('Parent',fig,'Position',[0.07,0.13,0.88,0.82]);
    hold(ax,'on');
    grid(ax,'on');
    axis(ax,'equal');
    axis(ax,'vis3d');

    xlabel(ax,'X_w / m','FontSize',WordSize);
    ylabel(ax,'Y_w / m','FontSize',WordSize);
    zlabel(ax,'Z_w / m','FontSize',WordSize);

    view(ax,3);
    rotate3d(fig,'on');
    zoom(fig,'on');

    all_xyz = [body_xyz;reshape(node_w,[],3)];
    all_xyz = all_xyz(all(isfinite(all_xyz),2),:);

    xyz_min = min(all_xyz,[],1);
    xyz_max = max(all_xyz,[],1);
    xyz_span = max(xyz_max-xyz_min);

    if ~isfinite(xyz_span) || xyz_span < 0.1
        xyz_span = 1;
    end

    margin = max(0.5,0.15*xyz_span);

    xlim(ax,[xyz_min(1)-margin,xyz_max(1)+margin]);
    ylim(ax,[xyz_min(2)-margin,xyz_max(2)+margin]);
    zlim(ax,[min(0,xyz_min(3))-0.2,xyz_max(3)+0.3]);

    full_xlim = xlim(ax);
    full_ylim = ylim(ax);
    full_zlim = zlim(ax);

    xl = xlim(ax);
    yl = ylim(ax);
    [gx,gy] = meshgrid(linspace(xl(1),xl(2),2),linspace(yl(1),yl(2),2));

    h_ground = surf(ax,gx,gy,zeros(size(gx)),'FaceColor',[0.75,0.75,0.75],'FaceAlpha',0.22,'EdgeColor',[0.55,0.55,0.55],'EdgeAlpha',0.35,'HandleVisibility','off');

    L0 = max(0.3,0.12*xyz_span);

    plot3(ax,0,0,0,'ko','MarkerFaceColor','k','MarkerSize',7,'HandleVisibility','off');
    quiver3(ax,0,0,0,L0,0,0,'r','LineWidth',2,'MaxHeadSize',0.8,'HandleVisibility','off');
    quiver3(ax,0,0,0,0,L0,0,'g','LineWidth',2,'MaxHeadSize',0.8,'HandleVisibility','off');
    quiver3(ax,0,0,0,0,0,L0,'b','LineWidth',2,'MaxHeadSize',0.8,'HandleVisibility','off');

    text(ax,L0,0,0,'X_w','Color','r','FontSize',WordSize);
    text(ax,0,L0,0,'Y_w','Color','g','FontSize',WordSize);
    text(ax,0,0,L0,'Z_w','Color','b','FontSize',WordSize);
    text(ax,0,0,0,' O_w','FontSize',WordSize);

    h_traj_all = plot3(ax,body_xyz(:,1),body_xyz(:,2),body_xyz(:,3),'k--','LineWidth',1);
    h_traj_now = plot3(ax,body_xyz(1,1),body_xyz(1,2),body_xyz(1,3),'k-','LineWidth',1);

    h_leg = gobjects(4,1);
    h_joint = gobjects(4,1);

    for leg = 1:4
        p = squeeze(node_w(1,leg,:,:));

        h_leg(leg) = plot3(ax,p(:,1),p(:,2),p(:,3),'-','Color',link_color,'LineWidth',2,'HandleVisibility','off');
        h_joint(leg) = plot3(ax,p(:,1),p(:,2),p(:,3),'LineStyle','none','Marker','o','MarkerSize',joint_marker_size,'MarkerEdgeColor',joint_color,'MarkerFaceColor',joint_color,'HandleVisibility','off');
    end

    h_force = gobjects(4,3);

    for leg = 1:4
        for force_i = 1:3
            force_origin = reshape(node_w(1,leg,force_i+1,:),1,3);
            force_vector = reshape(force_w(1,leg,force_i,:),1,3);

            h_force(leg,force_i) = quiver3(ax,force_origin(1),force_origin(2),force_origin(3),force_vector(1),force_vector(2),force_vector(3),'r','LineWidth',1.8,'MaxHeadSize',0.5,'AutoScale','off','HandleVisibility','off');
        end
    end

    body_order = [1,2,4,3,1];
    hip = squeeze(node_w(1,:,1,:));
    body_poly = hip(body_order,:);

    h_body = plot3(ax,body_poly(:,1),body_poly(:,2),body_poly(:,3),'-','Color',link_color,'LineWidth',3);
    h_base = plot3(ax,body_xyz(1,1),body_xyz(1,2),body_xyz(1,3),'o','MarkerEdgeColor',joint_color,'MarkerFaceColor',joint_color,'MarkerSize',joint_marker_size);

    [sphere_x,sphere_y,sphere_z] = sphere(24);

    h_collision = surf(ax,nan(size(sphere_x)),nan(size(sphere_y)),nan(size(sphere_z)),'EdgeColor','none','FaceColor',collision_color(1,:),'FaceAlpha',0.88,'Visible','off','HandleVisibility','off');

    h_title = title(ax,sprintf('CAPO Dog Motion in World Frame   t = %.3f s   frame = %d',t_vis(1),vis_idx(1)),'FontSize',WordSize);

    legend(ax,[h_traj_all,h_traj_now,h_body,h_base],{'Full trajectory','Current trajectory','Body links','Base position'},'Location','best');

    sld = uicontrol(fig,'Style','slider','Units','normalized','Position',[0.10,0.035,0.50,0.04],'Min',1,'Max',max(M,2),'Value',1);

    if M > 1
        sld.SliderStep = [1/(M-1),min(50/(M-1),1)];
    end

    btn_play = uicontrol(fig,'Style','togglebutton','Units','normalized','Position',[0.62,0.035,0.08,0.04],'String','Play');
    btn_full = uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.72,0.035,0.08,0.04],'String','全览');
    btn_follow = uicontrol(fig,'Style','togglebutton','Units','normalized','Position',[0.82,0.035,0.12,0.04],'String','视角跟随');

    state.M = M;
    state.node_w = node_w;
    state.force_w = force_w;
    state.body_xyz = body_xyz;
    state.yaw_vis = yaw_vis;
    state.vis_idx = vis_idx;
    state.t_vis = t_vis;
    state.body_order = body_order;

    state.ax = ax;
    state.h_ground = h_ground;
    state.h_leg = h_leg;
    state.h_joint = h_joint;
    state.h_force = h_force;
    state.h_body = h_body;
    state.h_base = h_base;
    state.h_traj_now = h_traj_now;
    state.h_title = h_title;
    state.h_collision = h_collision;

    state.sld = sld;
    state.btn_follow = btn_follow;

    state.full_xlim = full_xlim;
    state.full_ylim = full_ylim;
    state.full_zlim = full_zlim;

    state.follow_init_len = follow_init_len;
    state.follow_init_z_down = follow_init_z_down;
    state.follow_init_z_up = follow_init_z_up;
    state.follow_init_az = follow_init_az;
    state.follow_init_el = follow_init_el;

    state.collision_events = collision_events;
    state.collision_show_duration = collision_show_duration;
    state.collision_sphere_radius = collision_sphere_radius;
    state.sphere_x = sphere_x;
    state.sphere_y = sphere_y;
    state.sphere_z = sphere_z;
    state.collision_color = collision_color;

    setappdata(fig,'capo_motion_state',state);
    setappdata(ax,'follow_last_center',body_xyz(1,:));

    sld.Callback = @(src,~)plot_dog_motion_update_local(fig,round(get(src,'Value')));
    btn_play.Callback = @(src,~)plot_dog_motion_play_local(fig,src);
    btn_full.Callback = @(~,~)plot_dog_motion_full_view_local(fig);
    btn_follow.Callback = @(src,~)plot_dog_motion_toggle_follow_local(fig,src);

    plot_dog_motion_update_local(fig,1);
end


function plot_dog_motion_update_local(fig,ii)

    if ~isgraphics(fig) || ~isappdata(fig,'capo_motion_state')
        return;
    end

    state = getappdata(fig,'capo_motion_state');
    ii = max(1,min(state.M,ii));

    if isgraphics(state.sld)
        set(state.sld,'Value',ii);
    end

    for leg_i = 1:4
        p_i = squeeze(state.node_w(ii,leg_i,:,:));

        set(state.h_leg(leg_i),'XData',p_i(:,1),'YData',p_i(:,2),'ZData',p_i(:,3));
        set(state.h_joint(leg_i),'XData',p_i(:,1),'YData',p_i(:,2),'ZData',p_i(:,3));

        for force_i = 1:3
            force_origin_i = reshape(state.node_w(ii,leg_i,force_i+1,:),1,3);
            force_vector_i = reshape(state.force_w(ii,leg_i,force_i,:),1,3);

            set(state.h_force(leg_i,force_i),'XData',force_origin_i(1),'YData',force_origin_i(2),'ZData',force_origin_i(3),'UData',force_vector_i(1),'VData',force_vector_i(2),'WData',force_vector_i(3));
        end
    end

    hip_i = squeeze(state.node_w(ii,:,1,:));
    body_poly_i = hip_i(state.body_order,:);

    set(state.h_body,'XData',body_poly_i(:,1),'YData',body_poly_i(:,2),'ZData',body_poly_i(:,3));
    set(state.h_base,'XData',state.body_xyz(ii,1),'YData',state.body_xyz(ii,2),'ZData',state.body_xyz(ii,3));
    set(state.h_traj_now,'XData',state.body_xyz(1:ii,1),'YData',state.body_xyz(1:ii,2),'ZData',state.body_xyz(1:ii,3));

    collision_name = plot_dog_motion_collision_local(state,ii);

    if isempty(collision_name)
        title_text = sprintf('CAPO Dog Motion in World Frame   t = %.3f s   frame = %d',state.t_vis(ii),state.vis_idx(ii));
    else
        title_text = sprintf('CAPO Dog Motion in World Frame   t = %.3f s   frame = %d   collision = %s',state.t_vis(ii),state.vis_idx(ii),collision_name);
    end

    set(state.h_title,'String',title_text);

    if get(state.btn_follow,'Value') == 1
        plot_dog_motion_follow_center_local(state,ii);
    else
        setappdata(state.ax,'follow_last_center',state.body_xyz(ii,:));
    end

    drawnow limitrate;
end


function collision_name = plot_dog_motion_collision_local(state,ii)

    collision_name = '';

    if isempty(state.collision_events) || ~isgraphics(state.h_collision)
        if isgraphics(state.h_collision)
            set(state.h_collision,'Visible','off');
        end
        return;
    end

    current_time = state.t_vis(ii);
    event_time = state.collision_events(:,2);
    active_event = find(event_time <= current_time & current_time <= event_time+state.collision_show_duration,1,'last');

    if isempty(active_event)
        set(state.h_collision,'Visible','off');
        return;
    end

    code = state.collision_events(active_event,3);
    [center,color,collision_name] = plot_dog_motion_collision_center_local(state,ii,code);

    if any(~isfinite(center))
        set(state.h_collision,'Visible','off');
        collision_name = '';
        return;
    end

    radius = state.collision_sphere_radius;

    set(state.h_collision,'XData',center(1)+radius*state.sphere_x,'YData',center(2)+radius*state.sphere_y,'ZData',center(3)+radius*state.sphere_z,'FaceColor',color,'Visible','on');
end


function [center,color,name] = plot_dog_motion_collision_center_local(state,ii,code)

    center = [nan,nan,nan];
    color = [1,0,0];
    name = '';

    switch code
        case 1
            p0 = plot_dog_motion_extreme_joint_local(state,ii,0);
            p1 = plot_dog_motion_extreme_joint_local(state,ii,1);
            center = mean([p0;p1],1);
            color = state.collision_color(1,:);
            name = 'Front';

        case 2
            center = plot_dog_motion_extreme_joint_local(state,ii,0);
            color = state.collision_color(2,:);
            name = 'FrontLeft';

        case 3
            p0 = plot_dog_motion_extreme_joint_local(state,ii,0);
            p2 = plot_dog_motion_extreme_joint_local(state,ii,2);
            center = mean([p0;p2],1);
            color = state.collision_color(3,:);
            name = 'Left';

        case 4
            center = plot_dog_motion_extreme_joint_local(state,ii,2);
            color = state.collision_color(4,:);
            name = 'BackLeft';

        case 5
            p2 = plot_dog_motion_extreme_joint_local(state,ii,2);
            p3 = plot_dog_motion_extreme_joint_local(state,ii,3);
            center = mean([p2;p3],1);
            color = state.collision_color(5,:);
            name = 'Back';

        case 6
            center = plot_dog_motion_extreme_joint_local(state,ii,3);
            color = state.collision_color(6,:);
            name = 'BackRight';

        case 7
            p1 = plot_dog_motion_extreme_joint_local(state,ii,1);
            p3 = plot_dog_motion_extreme_joint_local(state,ii,3);
            center = mean([p1;p3],1);
            color = state.collision_color(7,:);
            name = 'Right';

        case 8
            center = plot_dog_motion_extreme_joint_local(state,ii,1);
            color = state.collision_color(8,:);
            name = 'FrontRight';
    end
end


function joint_world = plot_dog_motion_extreme_joint_local(state,ii,leg_id)

    p_world = squeeze(state.node_w(ii,leg_id+1,:,:));

    if size(p_world,1) ~= 4 || size(p_world,2) ~= 3
        joint_world = [nan,nan,nan];
        return;
    end

    body_p = state.body_xyz(ii,:);
    rel_world = p_world-body_p;

    yaw = state.yaw_vis(ii);

    if ~isfinite(yaw)
        yaw = 0;
    end

    cy = cos(yaw);
    sy = sin(yaw);

    x_body = cy*rel_world(:,1)+sy*rel_world(:,2);
    y_body = -sy*rel_world(:,1)+cy*rel_world(:,2);

    switch leg_id
        case 0
            score = x_body+y_body;
        case 1
            score = x_body-y_body;
        case 2
            score = -x_body+y_body;
        case 3
            score = -x_body-y_body;
        otherwise
            joint_world = [nan,nan,nan];
            return;
    end

    score(~all(isfinite(p_world),2)) = -inf;
    [best_score,best_node] = max(score);

    if ~isfinite(best_score)
        joint_world = [nan,nan,nan];
    else
        joint_world = p_world(best_node,:);
    end
end


function plot_dog_motion_toggle_follow_local(fig,btn_follow)

    if ~isgraphics(fig) || ~isappdata(fig,'capo_motion_state')
        return;
    end

    state = getappdata(fig,'capo_motion_state');
    ii = max(1,min(state.M,round(get(state.sld,'Value'))));

    if get(btn_follow,'Value') == 1
        c = state.body_xyz(ii,:);

        xlim(state.ax,[c(1)-state.follow_init_len,c(1)+state.follow_init_len]);
        ylim(state.ax,[c(2)-state.follow_init_len,c(2)+state.follow_init_len]);
        zlim(state.ax,[c(3)-state.follow_init_z_down-0.3,c(3)+state.follow_init_z_up]);

        view(state.ax,state.follow_init_az,state.follow_init_el);
        setappdata(state.ax,'follow_last_center',c);
        plot_dog_motion_update_ground_local(state.ax,state.h_ground);
    else
        setappdata(state.ax,'follow_last_center',state.body_xyz(ii,:));
    end

    drawnow;
end


function plot_dog_motion_follow_center_local(state,ii)

    c = state.body_xyz(ii,:);

    if isappdata(state.ax,'follow_last_center')
        last_c = getappdata(state.ax,'follow_last_center');
    else
        last_c = c;
    end

    xl = xlim(state.ax);
    yl = ylim(state.ax);
    zl = zlim(state.ax);

    dx = xl-last_c(1);
    dy = yl-last_c(2);
    dz = zl-last_c(3);

    if any(~isfinite(dx)) || diff(dx) <= 0
        dx = [-1,1];
    end

    if any(~isfinite(dy)) || diff(dy) <= 0
        dy = [-1,1];
    end

    if any(~isfinite(dz)) || diff(dz) <= 0
        dz = [-0.5,1.0];
    end

    xlim(state.ax,c(1)+dx);
    ylim(state.ax,c(2)+dy);
    zlim(state.ax,c(3)+dz);

    setappdata(state.ax,'follow_last_center',c);
    plot_dog_motion_update_ground_local(state.ax,state.h_ground);
end


function plot_dog_motion_full_view_local(fig)

    if ~isgraphics(fig) || ~isappdata(fig,'capo_motion_state')
        return;
    end

    state = getappdata(fig,'capo_motion_state');
    set(state.btn_follow,'Value',0);

    xlim(state.ax,state.full_xlim);
    ylim(state.ax,state.full_ylim);
    zlim(state.ax,state.full_zlim);

    plot_dog_motion_update_ground_local(state.ax,state.h_ground);
    drawnow;
end


function plot_dog_motion_update_ground_local(ax,h_ground)

    xl = xlim(ax);
    yl = ylim(ax);
    [gx,gy] = meshgrid(linspace(xl(1),xl(2),2),linspace(yl(1),yl(2),2));

    set(h_ground,'XData',gx,'YData',gy,'ZData',zeros(size(gx)));
end


function plot_dog_motion_play_local(fig,btn)

    if ~isgraphics(fig) || ~isappdata(fig,'capo_motion_state')
        return;
    end

    if get(btn,'Value') == 1
        set(btn,'String','Stop');
    else
        set(btn,'String','Play');
        return;
    end

    while isgraphics(fig) && isgraphics(btn) && get(btn,'Value') == 1
        state = getappdata(fig,'capo_motion_state');

        ii = round(get(state.sld,'Value'))+1;

        if ii > state.M
            ii = 1;
        end

        plot_dog_motion_update_local(fig,ii);
        pause(0.02);
    end

    if isgraphics(btn)
        set(btn,'Value',0);
        set(btn,'String','Play');
    end
end
