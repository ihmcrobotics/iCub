% Octave / MATLAB script for plotting data exported from the unit tests
% with speed and torque joint limits.

% The iCub 2.5 legs use the same electric motor and the same transmission
% for all the joints:
% - electric motor: MOOG - C2900584
% - transmission: HARMONIC DRIVE - CSD-17-100-2A-GR
% Nevertheless, the 'hip_pitch' joint has an additional cable transmission
% after the harmonic drive. The motor side pulley has a diameter of 50 [mm],
% while the joint side pulley has a diameter of 75 [mm].

clear all;

%% Hip_pitch additional cable transmission details
% The hip_pitch cable transmission uses a CarlStahl U7191517 steel cable.
% The minimum breaking load of the steel cable is 1590 [N]. 
% The diameter of the pulley of the hip_pitch joint is 75 [mm].
% The torque limit due to the steel cable is 1590*0.0375 = 59.625 [Nm].

HIP_PITCH_ADDITIONAL_JOINT_RATIO = 75/50;
HIP_PITCH_TENDON_TORQUE_LIMIT = 59.625;

%% Import simulation data
Simulation = 'iCub_dynamicWalk_2017-09-01.mat';
load(Simulation)

data = root.iCub;

%% Import nominal curves
filename = 'nominalLegJointCurves.txt';
delimiter = ' ';
formatSpec = '%f%f%[^\n\r]';

fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
fclose(fileID);

nominalLegJointCurves_velocity = dataArray{:, 1}';
nominalLegJointCurves_torque = dataArray{:, 2}';

%% Import peak curves
filename = 'peakLegJointCurves.txt';

fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
fclose(fileID);

peakLegJointCurves_velocity = dataArray{:, 1}';
peakLegJointCurves_torque = dataArray{:, 2}';

%% plot curves

joints = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};

figureTitle = strcat('Speed/Torque - ', ' Simulation: ', strrep(Simulation,'_','\_'));
figure('name', figureTitle);
for i=1:6
    ax = subplot(2,3,i);
    joint = joints{i};
    plotCurves(ax, joint, data, peakLegJointCurves_velocity, peakLegJointCurves_torque, ...
               nominalLegJointCurves_velocity, nominalLegJointCurves_torque, HIP_PITCH_ADDITIONAL_JOINT_RATIO, HIP_PITCH_TENDON_TORQUE_LIMIT)
end
text(-13.5,95, figureTitle, 'FontSize',14)

function plotCurves(ax, joint, data, peakLegJointCurves_velocity, peakLegJointCurves_torque, ...
                    nominalLegJointCurves_velocity, nominalLegJointCurves_torque, HIP_PITCH_ADDITIONAL_JOINT_RATIO, HIP_PITCH_TENDON_TORQUE_LIMIT)

    gridsize = 30;
    gridlines = 30;

    t = getfield(data, 't');
    q = getfield(data, strcat('q_', joint));
    qd = getfield(data, strcat('qd_', joint));
    tau = getfield(data, strcat('tau_', joint));

    % % compute the joints velocities norm and joints torques norm
    tau = abs(tau);
    qd  = abs(qd);
    
    if contains(joint, 'hip_pitch')
        peakLegJointCurves_velocity = peakLegJointCurves_velocity / HIP_PITCH_ADDITIONAL_JOINT_RATIO;
        nominalLegJointCurves_velocity = nominalLegJointCurves_velocity / HIP_PITCH_ADDITIONAL_JOINT_RATIO;
        peakLegJointCurves_torque = peakLegJointCurves_torque * HIP_PITCH_ADDITIONAL_JOINT_RATIO;
        nominalLegJointCurves_torque = nominalLegJointCurves_torque * HIP_PITCH_ADDITIONAL_JOINT_RATIO;
        isGreater = peakLegJointCurves_torque > HIP_PITCH_TENDON_TORQUE_LIMIT;
        peakLegJointCurves_torque(isGreater) = HIP_PITCH_TENDON_TORQUE_LIMIT;
    end

    qd_grid = linspace(min(qd), max([qd,  peakLegJointCurves_velocity]) + 0.5, gridsize);
    tau_grid = linspace(min(tau), max([tau, peakLegJointCurves_torque]) + 0.5, gridsize);
    intensity = zeros(gridsize, gridsize);

    for (i = 1:size(qd, 2))
      qdi = qd(i);
      taui = tau(i);
      [minVal qdIdx] = min(abs(qd_grid - qdi));
      [minVal tauIdx] = min(abs(tau_grid - taui));
      intensity(tauIdx, qdIdx) = intensity(tauIdx, qdIdx) + 1;
    end

    [xx,yy] = meshgrid(qd_grid, tau_grid);
    intensity = intensity ./ max(max(intensity));

    
    contourf(ax, xx, yy, intensity, gridlines, 'LineWidth', 1);
    xlabel('speed [rad/s]')
    ylabel('torque [Nm]')
    title(strcat(strrep(joint,'_','\_'), ': Speed/Torque'))
    hold on
    plot(ax, nominalLegJointCurves_velocity, nominalLegJointCurves_torque, 'r', 'LineWidth', 3)
    plot(ax, peakLegJointCurves_velocity, peakLegJointCurves_torque, 'r-.', 'LineWidth', 3)
    legend('simulation data','continuous range', 'peak')
end