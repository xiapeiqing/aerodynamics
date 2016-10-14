% save airfoil centerlinex centerliney wingtipx wingtipy twistx twisty
close all;
clear all;
load airfoil;
figure;plot(centerlinex,centerliney,'.');axis equal;title('centerline');grid on;
figure;plot(wingtipx,wingtipy,'.');axis equal;title('wingtip');grid on;
figure;plot(twistx,twisty,'.');axis equal;title('twist');ylabel('deg');grid on;
x=0:0.01:1;
figure;plot(x,(1-x.^2).^1.5,'.-');title('Gamma, airflow circulation');grid on;
figure;plot(x,1.5*(x.^2-0.5),'.-');title('Downwash');grid on;


