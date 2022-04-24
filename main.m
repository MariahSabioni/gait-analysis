%%% Basic read data and visualization for the course SG2804  
%%% Biomechanics of Human Movement at KTH - 2021
clear global;close all;clc

%% Set up the analysis
% select the motion to be loaded and visualized e.g. 'NormWalk', 'HighJump', 'LowJump'
motion = 'NormWalk';

%% Setup motion files
[data_trc, data_grf_s] = id.load_file(motion);
timestep = 1/100; %timestep = 1/frequency

%% Set initial and final frames
[frames_r, frames_l] = id.get_frames(motion);

%% Assign the uploaded table to variables in MATLAB
% Motion data
conversion = 1/1000; % data is originally in mm, it has to be divided by 1000 to have it in meters
[LTOE,LANKLE,LKNEE,LHIP,RTOE,RANKLE,RKNEE,RHIP,PELO,PELP,TRXO,TRXP] = id.get_points(conversion, data_trc);
% Force data
% Depending on the trial you have to use force plate 1 and 2 or 2 and 4.
% use force plate data 1 and 2 for walking / force plate data 2 and 4 for jumping
[FGR_l, COP_l, FGR_r, COP_r] = id.get_grd(conversion, motion, data_grf_s);

%% Compute the segment angles
foot_ang_r = id.get_seg_angle(RANKLE, RTOE);
foot_ang_l = id.get_seg_angle(LANKLE, LTOE);
shank_ang_r = id.get_seg_angle(RKNEE, RANKLE);
shank_ang_l = id.get_seg_angle(LKNEE, LANKLE);
thigh_ang_r = id.get_seg_angle(RHIP, RKNEE);
thigh_ang_l = id.get_seg_angle(LHIP, LKNEE);
pelvis_ang = id.get_seg_angle(PELP, PELO);
trunk_ang = id.get_seg_angle(TRXO, TRXP);

%% Trim and plot the segment angles
[foot_ang_r_trim] = id.trim (foot_ang_r, frames_r);
[foot_ang_l_trim] = id.trim (foot_ang_l, frames_l);
[shank_ang_r_trim] = id.trim (shank_ang_r, frames_r);
[shank_ang_l_trim] = id.trim (shank_ang_l, frames_l);
[thigh_ang_r_trim] = id.trim (thigh_ang_r, frames_r);
[thigh_ang_l_trim] = id.trim (thigh_ang_l, frames_l);
[pelvis_ang_r_trim] = id.trim (pelvis_ang, frames_r);
[pelvis_ang_l_trim] = id.trim (pelvis_ang, frames_l);
[trunk_ang_r_trim] = id.trim (trunk_ang, frames_r);
[trunk_ang_l_trim] = id.trim (trunk_ang, frames_l);
% id.plot_2_var(foot_ang_r_trim, foot_ang_l_trim, 'Foot Angle', 'dorsiflexion', 'plantarflexion', 'Degrees')
% id.plot_2_var(shank_ang_r_trim, shank_ang_l_trim, 'Shank Joint Angle', 'flexion', 'hyperextension', 'Degrees')
% id.plot_2_var(thigh_ang_r_trim, thigh_ang_l_trim, 'Thigh Joint Angle', 'flexion', 'hyperextension', 'Degrees')
id.plot_2_var(pelvis_ang_r_trim, pelvis_ang_l_trim, 'Pelvis Angle', 'ant. tilt', 'pos. tilt', 'Degrees')
id.plot_2_var(trunk_ang_r_trim, trunk_ang_l_trim, 'Trunk Angle', 'ant. tilt', 'pos. tilt', 'Degrees')

%% Compute the joint angles
ankle_ang_r = id.get_joint_angle(-foot_ang_r, -shank_ang_r, -85);
ankle_ang_l = id.get_joint_angle(-foot_ang_l, -shank_ang_l, -85);
knee_ang_r = id.get_joint_angle(shank_ang_r, thigh_ang_r, 0);
knee_ang_l = id.get_joint_angle(shank_ang_l, thigh_ang_l, 0);
hip_ang_r = id.get_joint_angle(-thigh_ang_r, -pelvis_ang, 0);
hip_ang_l = id.get_joint_angle(-thigh_ang_l, -pelvis_ang, 0);

%% Trim and plot the joint angles
[ankle_ang_r_trim] = id.trim (ankle_ang_r, frames_r);
[ankle_ang_l_trim] = id.trim (ankle_ang_l, frames_l);
[knee_ang_r_trim] = id.trim (knee_ang_r, frames_r);
[knee_ang_l_trim] = id.trim (knee_ang_l, frames_l);
[hip_ang_r_trim] = id.trim (hip_ang_r, frames_r);
[hip_ang_l_trim] = id.trim (hip_ang_l, frames_l);
id.plot_2_var(ankle_ang_r_trim, ankle_ang_l_trim, 'Ankle Joint Angle', 'dorsiflexion', 'plantarflexion', 'Degrees')
id.plot_2_var(knee_ang_r_trim, knee_ang_l_trim, 'Knee Joint Angle', 'flexion', 'hyperextension', 'Degrees')
id.plot_2_var(hip_ang_r_trim, hip_ang_l_trim, 'Hip Joint Angle', 'flexion', 'hyperextension', 'Degrees')

%% Compute angular velocity and acceleration of the segments
[foot_ang_vel_r, foot_ang_acc_r] = id.get_derivative(foot_ang_r, timestep);
[foot_ang_vel_l, foot_ang_acc_l] = id.get_derivative(foot_ang_l, timestep);
[shank_ang_vel_r, shank_ang_acc_r] = id.get_derivative(shank_ang_r, timestep);
[shank_ang_vel_l, shank_ang_acc_l] = id.get_derivative(shank_ang_l, timestep);
[thigh_ang_vel_r, thigh_ang_acc_r] = id.get_derivative(thigh_ang_r, timestep);
[thigh_ang_vel_l, thigh_ang_acc_l] = id.get_derivative(thigh_ang_l, timestep);
[pelvis_ang_vel, pelvis_ang_acc] = id.get_derivative(pelvis_ang, timestep);
[trunk_ang_vel, trunk_ang_acc] = id.get_derivative(trunk_ang, timestep);

%% Trim and plot angular velocity and acceleration of the segments
[foot_ang_vel_r_trim] = id.trim (foot_ang_vel_r, frames_r);
[foot_ang_vel_l_trim] = id.trim (foot_ang_vel_l, frames_l);
[foot_ang_acc_r_trim] = id.trim (foot_ang_acc_r, frames_r);
[foot_ang_acc_l_trim] = id.trim (foot_ang_acc_l, frames_l);
[shank_ang_vel_r_trim] = id.trim (shank_ang_vel_r, frames_r);
[shank_ang_vel_l_trim] = id.trim (shank_ang_vel_l, frames_l);
[shank_ang_acc_r_trim] = id.trim (shank_ang_acc_r, frames_r);
[shank_ang_acc_l_trim] = id.trim (shank_ang_acc_l, frames_l);
[thigh_ang_vel_r_trim] = id.trim (thigh_ang_vel_r, frames_r);
[thigh_ang_vel_l_trim] = id.trim (thigh_ang_vel_l, frames_l);
[thigh_ang_acc_r_trim] = id.trim (thigh_ang_acc_r, frames_r);
[thigh_ang_acc_l_trim] = id.trim (thigh_ang_acc_l, frames_l);
[pelvis_ang_vel_r_trim] = id.trim (pelvis_ang_vel, frames_r);
[pelvis_ang_vel_l_trim] = id.trim (pelvis_ang_vel, frames_l);
[pelvis_ang_acc_r_trim] = id.trim (pelvis_ang_acc, frames_r);
[pelvis_ang_acc_l_trim] = id.trim (pelvis_ang_acc, frames_l);
[trunk_ang_vel_r_trim] = id.trim (trunk_ang_vel, frames_r);
[trunk_ang_vel_l_trim] = id.trim (trunk_ang_vel, frames_l);
[trunk_ang_acc_r_trim] = id.trim (trunk_ang_acc, frames_r);
[trunk_ang_acc_l_trim] = id.trim (trunk_ang_acc, frames_l);

% id.plot_2_var(foot_ang_vel_r_trim, foot_ang_vel_l_trim, 'Foot Angular Velocity', 'dorsiflexion', 'plantarflexion', 'Degrees/s')
% id.plot_2_var(shank_ang_vel_r_trim, shank_ang_vel_l_trim, 'Shank Angular Velocity', 'flexion', 'hyperextension', 'Degrees/s')
% id.plot_2_var(thigh_ang_vel_r_trim, thigh_ang_vel_l_trim, 'Thigh Angular Velocity', 'flexion', 'hyperextension', 'Degrees/s')
% id.plot_2_var(pelvis_ang_vel_r_trim, pelvis_ang_vel_l_trim, 'Pelvis Angular Velocity', 'ant tilt', 'pos. tilt', 'Degrees/s')
% id.plot_2_var(trunk_ang_vel_r_trim, trunk_ang_vel_l_trim, 'Trunk Angular Velocity', 'ant tilt', 'pos. tilt', 'Degrees/s')

% id.plot_2_var(foot_ang_acc_r_trim, foot_ang_acc_l_trim, 'Foot Angular Acceleration', 'dorsiflexion', 'plantarflexion', 'Degrees/s²')
% id.plot_2_var(shank_ang_acc_r_trim, shank_ang_acc_l_trim, 'Shank Angular Acceleration', 'flexion', 'hyperextension', 'Degrees/s²')
% id.plot_2_var(thigh_ang_acc_r_trim, thigh_ang_acc_l_trim, 'Thigh Angular Acceleration', 'flexion', 'hyperextension', 'Degrees/s²')
% id.plot_2_var(pelvis_ang_acc_r_trim, pelvis_ang_acc_l_trim, 'Pelvis Angular Acceleration', 'ant tilt', 'pos. tilt', 'Degrees/s²')
% id.plot_2_var(trunk_ang_acc_r_trim, trunk_ang_acc_l_trim, 'Trunk Angular Acceleration', 'ant tilt', 'pos. tilt', 'Degrees/s²')

%% Compute angular velocity and acceleration of the joints
[ankle_ang_vel_r, ankle_ang_acc_r] = id.get_derivative(ankle_ang_r, timestep);
[ankle_ang_vel_l, ankle_ang_acc_l] = id.get_derivative(ankle_ang_l, timestep);
[knee_ang_vel_r, knee_ang_acc_r] = id.get_derivative(knee_ang_r, timestep);
[knee_ang_vel_l, knee_ang_acc_l] = id.get_derivative(knee_ang_l, timestep);
[hip_ang_vel_r, hip_ang_acc_r] = id.get_derivative(hip_ang_r, timestep);
[hip_ang_vel_l, hip_ang_acc_l] = id.get_derivative(hip_ang_l, timestep);

%% Trim and plot angular velocity and acceleration of the joints
[ankle_ang_vel_r_trim] = id.trim (ankle_ang_vel_r, frames_r);
[ankle_ang_vel_l_trim] = id.trim (ankle_ang_vel_l, frames_l);
[knee_ang_vel_r_trim] = id.trim (knee_ang_vel_r, frames_r);
[knee_ang_vel_l_trim] = id.trim (knee_ang_vel_l, frames_l);
[hip_ang_vel_r_trim] = id.trim (hip_ang_vel_r, frames_r);
[hip_ang_vel_l_trim] = id.trim (hip_ang_vel_l, frames_l);

% id.plot_2_var(ankle_ang_vel_r_trim, ankle_ang_vel_l_trim, 'Ankle Joint Angular Velocity', 'dorsiflexion', 'plantarflexion', 'Degrees/s')
% id.plot_2_var(knee_ang_vel_r_trim, knee_ang_vel_l_trim, 'Knee Joint Angular Velocity', 'flexion', 'hyperextension', 'Degrees/s')
% id.plot_2_var(hip_ang_vel_r_trim, hip_ang_vel_l_trim, 'Hip Joint Angular Velocity', 'flexion', 'hyperextension', 'Degrees/s')
% 
% id.plot_2_var(ankle_ang_acc_r_trim, ankle_ang_acc_l_trim, 'Ankle Joint Angular Velocity', 'dorsiflexion', 'plantarflexion', 'Degrees/s')
% id.plot_2_var(knee_ang_acc_r_trim, knee_ang_acc_l_trim, 'Knee Joint Angular Velocity', 'flexion', 'hyperextension', 'Degrees/s')
% id.plot_2_var(hip_ang_acc_r_trim, hip_ang_acc_l_trim, 'Hip Joint Angular Velocity', 'flexion', 'hyperextension', 'Degrees/s')

%% Calculate the segment length
% height = 1.752;
% foot_len = height*0.152;
% shank_len = height*(0.285-0.039);
% thigh_len = height*(0.530-0.285);
foot_len = id.get_seg_len(RTOE, RANKLE, LTOE, LANKLE, frames_r, frames_l);
shank_len = id.get_seg_len(RANKLE, RKNEE, LANKLE, LKNEE, frames_r, frames_l);
thigh_len = id.get_seg_len(RKNEE, RHIP, LKNEE, LHIP, frames_r, frames_l);

%% Set constants (segment masses, Ig, COM, ...)
g = 9.81;
bm = 65.2;
foot_m = 0.0145*bm;
shank_m = 0.0465*bm;
thigh_m = 0.1*bm;
pelvis_m = 0.142*bm;

foot_Ig = foot_m *(0.475*foot_len)^2;
shank_Ig = shank_m *(0.302*shank_len)^2;
thigh_Ig = thigh_m *(0.323*thigh_len)^2;

foot_com = 0.5;
shank_com = 0.433;
thigh_com = 0.433;

%% calculate segment center of mass position in each frame
foot_com_r = id.get_com(RTOE, RANKLE, foot_com);
foot_com_l = id.get_com(LTOE, LANKLE, foot_com);
shank_com_r = id.get_com(RANKLE, RKNEE, shank_com);
shank_com_l = id.get_com(LANKLE, LKNEE, shank_com);
thigh_com_r = id.get_com(RKNEE, RHIP, thigh_com);
thigh_com_l = id.get_com(LKNEE, LHIP, thigh_com);

%% Calculate segment center of mass acceleration
foot_com_acc_r = id.get_seg_acc(foot_com_r, timestep);
foot_com_acc_l = id.get_seg_acc(foot_com_l, timestep);
shank_com_acc_r = id.get_seg_acc(shank_com_r, timestep);
shank_com_acc_l = id.get_seg_acc(shank_com_l, timestep);
thigh_com_acc_r = id.get_seg_acc(thigh_com_r, timestep);
thigh_com_acc_l = id.get_seg_acc(thigh_com_l, timestep);

%% Convert segment angular acceleration to rad/s²
% foot_ang_acc_r = deg2rad(foot_ang_acc_r);
% foot_ang_acc_l = deg2rad(foot_ang_acc_l);
% shank_ang_acc_r = deg2rad(shank_ang_acc_r);
% shank_ang_acc_l = deg2rad(shank_ang_acc_l);
% thigh_ang_acc_r = deg2rad(thigh_ang_acc_r);
% thigh_ang_acc_l = deg2rad(thigh_ang_acc_l);

%% Trim FGR, COP, joint positions, joint anf vel, joint ang acc, COM positions, COM acc
%FP2 and COP2 correspond to right side
%FP1 and COP1 correspond to left side
Fg_r = id.trim(FGR_r, frames_r);
Fg_l = id.trim(FGR_l, frames_l);

COP_r = id.trim(COP_r, frames_r);
COP_l = id.trim(COP_l, frames_l);

RTOE = id.trim(RTOE, frames_r);
LTOE = id.trim(LTOE, frames_l);
RANKLE = id.trim(RANKLE, frames_r);
LANKLE = id.trim(LANKLE, frames_l);
RKNEE = id.trim(RKNEE, frames_r);
LKNEE = id.trim(LKNEE, frames_l);
RHIP = id.trim(RHIP, frames_r);
LHIP = id.trim(LHIP, frames_l);

foot_com_r = id.trim(foot_com_r, frames_r);
foot_com_l = id.trim(foot_com_l, frames_l);
shank_com_r = id.trim(shank_com_r, frames_r);
shank_com_l = id.trim(shank_com_l, frames_l);
thigh_com_r = id.trim(thigh_com_r, frames_r);
thigh_com_l = id.trim(thigh_com_l, frames_l);

foot_com_acc_r = id.trim(foot_com_acc_r, frames_r);
foot_com_acc_l = id.trim(foot_com_acc_l, frames_l);
shank_com_acc_r = id.trim(shank_com_acc_r, frames_r);
shank_com_acc_l = id.trim(shank_com_acc_l, frames_l);
thigh_com_acc_r = id.trim(thigh_com_acc_r, frames_r);
thigh_com_acc_l = id.trim(thigh_com_acc_l, frames_l);

foot_ang_acc_r = deg2rad(id.trim(foot_ang_acc_r, frames_r));
foot_ang_acc_l = deg2rad(id.trim(foot_ang_acc_l, frames_l));
shank_ang_acc_r = deg2rad(id.trim(shank_ang_acc_r, frames_r));
shank_ang_acc_l = deg2rad(id.trim(shank_ang_acc_l, frames_l));
thigh_ang_acc_r = deg2rad(id.trim(thigh_ang_acc_r, frames_r));
thigh_ang_acc_l = deg2rad(id.trim(thigh_ang_acc_l, frames_l));

ankle_ang_vel_r = deg2rad(id.trim(ankle_ang_vel_r,frames_r));
ankle_ang_vel_l = deg2rad(id.trim(ankle_ang_vel_l,frames_l));
knee_ang_vel_r = deg2rad(id.trim(knee_ang_vel_r,frames_r));
knee_ang_vel_l = deg2rad(id.trim(knee_ang_vel_l,frames_l));
hip_ang_vel_r = deg2rad(id.trim(hip_ang_vel_r,frames_r));
hip_ang_vel_l = deg2rad(id.trim(hip_ang_vel_l,frames_l));

%% Compute and plot joint moments
M0_r = zeros(size(foot_ang_acc_r,1),1);
M0_l = zeros(size(foot_ang_acc_l,1),1);
[ankle_moment_r, ankle_force_r] = id.get_joint_moment(foot_m, foot_Ig, foot_ang_acc_r, foot_com_acc_r, foot_com_r, M0_r, Fg_r, COP_r, RANKLE);
[ankle_moment_l, ankle_force_l] = id.get_joint_moment(foot_m, foot_Ig, foot_ang_acc_l, foot_com_acc_l, foot_com_l, M0_l, Fg_l, COP_l, LANKLE);
id.plot_2_var(-ankle_moment_r/bm, -ankle_moment_l/bm, 'Ankle Joint Moment', 'plantarflexor', 'dorsiflexor', 'N⋅m/kg')

[knee_moment_r, knee_force_r] = id.get_joint_moment(shank_m, shank_Ig, -shank_ang_acc_r, shank_com_acc_r, shank_com_r, -ankle_moment_r, -ankle_force_r, RANKLE, RKNEE);
[knee_moment_l, knee_force_l] = id.get_joint_moment(shank_m, shank_Ig, -shank_ang_acc_l, shank_com_acc_l, shank_com_l, -ankle_moment_l, -ankle_force_l, LANKLE, LKNEE);
id.plot_2_var(knee_moment_r/bm, knee_moment_l/bm, 'Knee Joint Moment', 'extensor', 'flexor', 'N⋅m/kg')

[hip_moment_r, hip_force_r] = id.get_joint_moment(thigh_m, thigh_Ig, thigh_ang_acc_r, thigh_com_acc_r, thigh_com_r, -knee_moment_r, -knee_force_r, RKNEE, RHIP);
[hip_moment_l, hip_force_l] = id.get_joint_moment(thigh_m, thigh_Ig, thigh_ang_acc_l, thigh_com_acc_l, thigh_com_l, -knee_moment_l, -knee_force_l, LKNEE, LHIP);
id.plot_2_var(-hip_moment_r/bm, -hip_moment_l/bm, 'Hip Joint Moment', 'extensor', 'flexor', 'N⋅m/kg')

%% Compute and plot joint powers
ankle_power_r = id.get_power(ankle_moment_r, ankle_ang_vel_r);
ankle_power_l = id.get_power(ankle_moment_l, ankle_ang_vel_l);
id.plot_2_var(-ankle_power_r, -ankle_power_l, 'Ankle Joint Power', 'plantarflexor', 'dorsiflexor', 'W')

knee_power_r = id.get_power(knee_moment_r, knee_ang_vel_r);
knee_power_l = id.get_power(knee_moment_l, knee_ang_vel_l);
id.plot_2_var(knee_power_r, knee_power_l, 'Knee Joint Power', 'extensor', 'flexor', 'W')

hip_power_r = id.get_power(-hip_moment_r, hip_ang_vel_r);
hip_power_l = id.get_power(-hip_moment_l, hip_ang_vel_l);
id.plot_2_var(-hip_power_r, -hip_power_l, 'Hip Joint Power', 'extensor', 'flexor', 'W')
