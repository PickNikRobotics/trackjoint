% Run this script from the directory that holds the saved .csv datafiles

clear all; close all; clc

plot_one_joint('output_joint1.csv', 'Joint 1');
plot_one_joint('output_joint2.csv', 'Joint 2');
plot_one_joint('output_joint3.csv', 'Joint 3');

function plot_one_joint(filename, plot_title)

  data = dlmread(filename,' ');

  time = data(:, 1);
  position = data(:, 2);
  velocity = data(:, 3);
  acceleration = data(:, 4);
  jerk = data(:, 5);

  figure
  hold on
  subplot(4,1,1)
  plot(time, position)
  xlabel('Time')
  ylabel('Position')
  title(plot_title)

  subplot(4,1,2)
  plot(time, velocity)
  xlabel('Time')
  ylabel('Velocity')

  subplot(4,1,3)
  plot(time, acceleration)
  xlabel('Time')
  ylabel('Acceleration')

  subplot(4,1,4)
  plot(time, jerk)
  xlabel('Time')
  ylabel('Jerk')
end
