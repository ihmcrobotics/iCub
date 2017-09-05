# Octave / MATLAB script for plotting data exported from the unit tests.

clear all;

load 'iCub_dynamicWalk_2017-09-01.mat';
joint = 'r_knee';

gridsize = 20;
gridlines = 30;

data = root.iCub;

t = getfield(data, 't');
q = getfield(data, strcat('q_', joint));
qd = getfield(data, strcat('qd_', joint));
tau = getfield(data, strcat('tau_', joint));

qd_grid = linspace(min(qd) - 1.0, max(qd) + 1.0, gridsize);
tau_grid = linspace(min(tau) - 1.0, max(tau) + 1.0, gridsize);
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

figure('name', strcat(joint, ': Speed/Torque'));
contourf(xx, yy, intensity, gridlines, 'LineWidth', 0);
xlabel('speed')
ylabel('torque')

figure('name', strcat(joint, ': Joint Position'));
plot(t, q, 'LineWidth', 2)
xlabel('time')
ylabel('angle')
xlim([min(t), max(t)])