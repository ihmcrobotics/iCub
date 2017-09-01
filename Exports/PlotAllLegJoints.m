# Octave / MATLAB script for plotting data exported from the unit tests.

clear all;

load 'iCub_staticWalk_2017-09-01.mat';
joints = ['hip_pitch'; 'hip_roll'; 'hip_yaw'; 'knee'; 'ankle_pitch'; 'ankle_roll'];

data = root.iCub;

figure
hold on

for (jointIdx = 1:size(joints, 1))
  t = getfield(data, 't');
  q = getfield(data, strcat('q_r_', joints(jointIdx, :)));
  qd = getfield(data, strcat('qd_r_', joints(jointIdx, :)));
  tau = getfield(data, strcat('tau_r_', joints(jointIdx, :)));
  
  subplot(size(joints, 1), 2, 1 + (jointIdx-1) * 2)
  hold on
  plot(t, q * 180 / pi, 'r', 'LineWidth', 2);
  xlim([t(1), t(end)])
  title([joints(jointIdx, :), ' joint positions'], 'Interpreter', 'none')
  ylabel('q [deg]')
  xlabel('time [s]')

  subplot(size(joints, 1), 2, 2 + (jointIdx-1) * 2)
  plot(t, tau, 'r', 'LineWidth', 2);
  hold on
  xlim([t(1), t(end)])
  title([joints(jointIdx, :), ' joint torques'], 'Interpreter', 'none')
  ylabel('tau [Nm]')
  xlabel('time [s]')
  
  q = getfield(data, strcat('q_l_', joints(jointIdx, :)));
  qd = getfield(data, strcat('qd_l_', joints(jointIdx, :)));
  tau = getfield(data, strcat('tau_l_', joints(jointIdx, :)));
  
  subplot(size(joints, 1), 2, 1 + (jointIdx-1) * 2)
  plot(t, q * 180 / pi, 'b','LineWidth', 2);

  subplot(size(joints, 1), 2, 2 + (jointIdx-1) * 2)
  plot(t, tau, 'b', 'LineWidth', 2);
end