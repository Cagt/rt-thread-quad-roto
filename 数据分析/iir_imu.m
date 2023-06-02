clear all


acc_m = readtable("fucking.csv");
m1 = table2array(acc_m(:,2));
Fs = 1000; %20hz
N = 15150;

t_step = 1/Fs;
t = [0:t_step:(15150-1)*t_step];


y1 = fft(m1);


times = [0:1:15150-1];
Freq = times * Fs / N
abs_y1 = abs(y1);
abs_y1 = abs_y1 / (N/2);

plot(Freq,abs_y1,'r')