clear all
close all
measurements = importdata("rpm.txt");
speed = measurements(:,2);
speed_len = length(speed);
last_time = speed_len * 0.01;
length(speed)

t = 0:0.01:last_time;
length(t)
t = t(1:end-1)
plot(t(1:20),speed(1:20));
hold on
sk = measurements(:,1)
figure(1)
plot(t(1:20),sk(1:20));

in_data = iddata(speed(1:end),sk(1:end),0.01)
silnik_tf = tfest(in_data,1,0,'Ts',0.01)
silnik_laplace = d2c(silnik_tf)
figure(2)
step(silnik_laplace)

pidC = pidtune(silnik_laplace,'PID')

T = feedback(silnik_laplace*pidC,1)
figure(3)
step(T)
