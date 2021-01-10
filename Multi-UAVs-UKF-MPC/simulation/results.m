%%  call target function
[xt,yt,zt] = target_function;

%%  results
tt1 = (linspace(t1(1),t1(end),100))';
figure
for i = 1:length(tt1)
    [x_obs,y_obs,z_obs,xo_1,yo_1,zo_1,xo_2,yo_2,zo_2,xo_3,yo_3,zo_3] = obstacle_function(tt1(i));
    clf
    hold on
    mesh(xo_1,yo_1,zo_1)
    mesh(xo_2,yo_2,zo_2)
    mesh(xo_3,yo_3,zo_3)
    X1 = interp1(t1,[x1(:,1),x1(:,2),x1(:,3)],tt1(1:i));
    X2 = interp1(t1,[x1(:,4),x1(:,5),x1(:,6)],tt1(1:i));
    X3 = interp1(t1,[x1(:,7),x1(:,8),x1(:,9)],tt1(1:i));
    plot3(X1(:,1),X1(:,2),X1(:,3),'b--','linewidth',3)
    plot3(X2(:,1),X2(:,2),X2(:,3),'r--','linewidth',3)
    plot3(X3(:,1),X3(:,2),X3(:,3),'g--','linewidth',3)
    plot3(xt,yt,zt,'k.','markersize',50)
    view(-60,20), axis equal
    xlabel('x'), ylabel('y'), zlabel('z'), title('UKF')
    pause(1)
end

tt2 = (linspace(t2(1),t2(end),100))';
figure
for i = 1:length(tt2)
    [x_obs,y_obs,z_obs,xo_1,yo_1,zo_1,xo_2,yo_2,zo_2,xo_3,yo_3,zo_3] = obstacle_function(tt2(i));
    clf
    hold on
    mesh(xo_1,yo_1,zo_1)
    mesh(xo_2,yo_2,zo_2)
    mesh(xo_3,yo_3,zo_3)
    X4 = interp1(t2,[x2(:,1),x2(:,2),x2(:,3)],tt2(1:i));
    X5 = interp1(t2,[x2(:,4),x2(:,5),x2(:,6)],tt2(1:i));
    X6 = interp1(t2,[x2(:,7),x2(:,8),x2(:,9)],tt2(1:i));
    plot3(X4(:,1),X4(:,2),X4(:,3),'b--','linewidth',3)
    plot3(X5(:,1),X5(:,2),X5(:,3),'r--','linewidth',3)
    plot3(X6(:,1),X6(:,2),X6(:,3),'g--','linewidth',3)
    plot3(xt,yt,zt,'k.','markersize',50)
    view(-60,20), axis equal
    xlabel('x'), ylabel('y'), zlabel('z'), title('EKF')
    pause(1)
end

se1_ukf = trapz(t1',sum([x1(:,1),x1(:,2),x1(:,3)]' - [xt,yt,zt]').^2);
se2_ukf = trapz(t1',sum([x1(:,4),x1(:,5),x1(:,6)]' - [xt,yt,zt]').^2);
se3_ukf = trapz(t1',sum([x1(:,7),x1(:,8),x1(:,9)]' - [xt,yt,zt]').^2);
ae1_ukf = trapz(t1',abs(sum([x1(:,1),x1(:,2),x1(:,3)]' - [xt,yt,zt]')));
ae2_ukf = trapz(t1',abs(sum([x1(:,4),x1(:,5),x1(:,6)]' - [xt,yt,zt]')));
ae3_ukf = trapz(t1',abs(sum([x1(:,7),x1(:,8),x1(:,9)]' - [xt,yt,zt]')));
se1_ekf = trapz(t2',sum([x2(:,1),x2(:,2),x2(:,3)]' - [xt,yt,zt]').^2);
se2_ekf = trapz(t2',sum([x2(:,4),x2(:,5),x2(:,6)]' - [xt,yt,zt]').^2);
se3_ekf = trapz(t2',sum([x2(:,7),x2(:,8),x2(:,9)]' - [xt,yt,zt]').^2);
ae1_ekf = trapz(t2',abs(sum([x2(:,1),x2(:,2),x2(:,3)]' - [xt,yt,zt]')));
ae2_ekf = trapz(t2',abs(sum([x2(:,4),x2(:,5),x2(:,6)]' - [xt,yt,zt]')));
ae3_ekf = trapz(t2',abs(sum([x2(:,7),x2(:,8),x2(:,9)]' - [xt,yt,zt]')));
ISE_1 = [se1_ukf;se1_ekf];
ISE_2 = [se2_ukf;se2_ekf];
ISE_3 = [se3_ukf;se3_ekf];
IAE_1 = [ae1_ukf;ae1_ekf];
IAE_2 = [ae2_ukf;ae2_ekf];
IAE_3 = [ae3_ukf;ae3_ekf];
IntegratedSquaredError = {'UKF';'EKF'};
T1 = table(IntegratedSquaredError,ISE_1,ISE_2,ISE_3);
IntegratedAbsoluteError = {'UKF';'EKF'};
T2 = table(IntegratedAbsoluteError,IAE_1,IAE_2,IAE_3);
disp(T1)
disp(T2)