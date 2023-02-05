lambda = 0:0.01:4*pi;
ang = [0:0.01:2*pi];

% v = VideoWriter('newfile.avi');
% open(v)

figure();
plot(r*cos(lambda), r*sin(lambda),'--r','color','green','linewidth',4);
hold on
plot(x1_plot,x2_plot,'r','color','red','linewidth',2);
xlabel('$x_{1}(m)$','FontSize',16,'Interpreter','latex')
ylabel('$y_{1}(m)$','FontSize',16,'Interpreter','latex')
grid on;
for i=1:length(eta0)
    plot(x_obs(i, 1) + epsilon2*cos(ang), x_obs(i, 2) + epsilon2*sin(ang), 'LineWidth',8)
end
k = 1;
Frames = struct('cdata',[],'colormap',[]);



% writeVideo(v,Frames)
% close(v)
%%

figure();
plot(r*cos(lambda),r*sin(lambda),'--r','color','green','linewidth',4);
hold on;
plot(x1_plot,x2_plot,'r','color','red','linewidth',2);
% legend({'$\chi(0)$','$\gamma$','$h(\chi)$'},'FontSize',14,'Interpreter','latex')
xlabel('$x_{1}(m)$','FontSize',16,'Interpreter','latex')
ylabel('$y_{1}(m)$','FontSize',16,'Interpreter','latex')
grid on;


plot(x0(1)+epsilon1*cos(ang),x0(2)+epsilon1*sin(ang))

%%%%%%%%%%%%%%%%%%%%%%%% visuals %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for  i = 1:T/dt
    if (mod(i,200) == 0 || i == 1)
        
        [x_rotated,y_rotated] = robot_display(x1_plot(i),x2_plot(i),x3_plot(i));
        
        if (i == 1)
            fill(x_rotated, y_rotated,'g');
        else
            plot(x_rotated, y_rotated, 'r');
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%


figure();
plot(time(1:end-1),eta1_plot(1:end-1),'r','color','red','linewidth',2)
hold on;
plot(time(1:end-1),eta2_plot(1:end-1),'--r','color','green','linewidth',2)
grid on;
legend({'$\eta_{1}$','$\eta_{2}$'},'FontSize',14,'Interpreter','latex')
xlabel('$time(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_{1},\eta_{2}$','FontSize',16,'Interpreter','latex')


figure();
plot(time(1:end-1),xi1_plot(1:end-1),'r','color','red','linewidth',2)
hold on;
plot(time(1:end-1),xi2_plot(1:end-1),'--r','color','green','linewidth',2)
grid on;

legend({'$\xi_{1}$','$\xi_{2}$'},'FontSize',14,'Interpreter','latex')
xlabel('$time(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\xi_{1},\xi_{2}$','FontSize',16,'Interpreter','latex')
