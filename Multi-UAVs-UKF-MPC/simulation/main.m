%%  ALLAH

%%  neccessary commands
clc
clear
close all

%%  solve equations
[t1,x1,u1] = uav_model_ukf;
[t2,x2,u2] = uav_model_ekf;

%%  results
results