function plot_one_joint(filename, plot_title)

  data = dlmread(filename,' ');

  time = data(:, 1);
  position = data(:, 2);
  velocity = data(:, 3);
  acceleration = data(:, 4);

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
  plot(time, velocity)
  xlabel('Time')
  ylabel('Acceleration')
end
