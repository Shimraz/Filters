% generate 100 data for position and corresponding velocity
n_pos = [];
t = 1:1:100;
for i = 1:100
   n_pos = [n_pos, i * 0.2]; % because velocity is 0.2
end
d = robot_data(100);

% applying filter
S = 0.1;
x_avg = zeros(1,100);
for i = 1:100
   x_cal = n_pos(i) + 0.2;
   r = d(i) - x_cal;
   x_avg(i) = x_cal + S * r;
end    

% plot
plot(t,n_pos)
hold on
plot(t,d)
hold on
plot(t, x_avg)
legend({'Position: Sensor data','Position: generated','Filtered position'}, 'Location',"best")
xlabel('time (seconds)')
ylabel('position')
