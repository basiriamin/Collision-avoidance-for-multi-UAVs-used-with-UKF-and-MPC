%%  ALLAH

function [x_obs,y_obs,z_obs,xo_1,yo_1,zo_1,xo_2,yo_2,zo_2,xo_3,yo_3,zo_3] = obstacle_function(t)
%%  dynamic position
T = [0,25,75,125,175,200];
Y = [0,500,-500,500,-500,0];

%%  obstacle 1
[xo_1,yo_1,zo_1] = sphere(100);
xo_1 = 50*(xo_1) + 0;
yo_1 = 50*(yo_1) + 0 + interp1(T',Y',t);
zo_1 = 50*(zo_1) + 350;

%%  obstacle 2
[xo_2,yo_2,zo_2] = sphere(100);
xo_2 = 50*(xo_2) - 500;
yo_2 = 50*(yo_2) + interp1(T',Y',t);
zo_2 = 50*(zo_2) + 350;

%%  obstacle 3
[xo_3,yo_3,zo_3] = sphere(100);
xo_3 = 50*(xo_3) - 1000;
yo_3 = 50*(yo_3) - interp1(T',Y',t);
zo_3 = 50*(zo_3) + 0;

%%  total obstacles
x_obs = [reshape(xo_1,1,size(xo_1,1)*size(xo_1,1)),...
    reshape(xo_2,1,size(xo_2,1)*size(xo_2,1)),...
    reshape(xo_3,1,size(xo_3,1)*size(xo_3,1))];
y_obs = [reshape(yo_1,1,size(yo_1,1)*size(yo_1,1)),...
    reshape(yo_2,1,size(yo_2,1)*size(yo_2,1)),...
    reshape(yo_3,1,size(yo_3,1)*size(yo_3,1))];
z_obs = [reshape(zo_1,1,size(zo_1,1)*size(zo_1,1)),...
    reshape(zo_2,1,size(zo_2,1)*size(zo_2,1)),...
    reshape(zo_3,1,size(zo_3,1)*size(zo_3,1))];
end