%% plot barrier functions
function_1 = [];
function_2 = [];
n_iter = length(x1_plot);
n_obs = size(xs_obs);
n_obs = n_obs(1);
bar_f = zeros(n_iter, n_obs);
for i = 1 : n_iter
    for j = 1 : n_obs
        bar_f(i, j) = epsilon2 - sqrt( (x1_plot(i) - xs_obs(j, 1))^2 + (x2_plot(i) - xs_obs(j, 2))^2 );
    end
end
time = 0:n_iter - 1;
time = time * dt;

font_size = 18;
figure
hold on
plot(time, bar_f, 'LineWidth', 2)
plot(time, bar_f(:, 1)*0,'--', 'LineWidth', 2)
set(gca,'FontSize', font_size)
grid 

xlim([0 20])

xlabel('$Time\,(s)$', 'Interpreter', 'latex')
ylabel('$CBF$', 'Interpreter', 'latex')
legend('$h_1$', 'Limit', 'NumColumns', 3, 'Interpreter', 'latex')
%% plot cbf multiple experiments
function_1 = [];
function_2 = [];
names = {'one_obstacle_2', 'one_obstacle_3', 'one_obstacle_4', 'one_obstacle_5', 'one_obstacle_6'};

n_sims = length(names);
n_iter = length(x1_plot);
bar_f = zeros(n_iter, n_sims);
for j = 1 : n_sims
    load(names{j})
    n_iter = length(x1_plot);
    for i = 1 : n_iter
        bar_f(i, j) = epsilon2 - sqrt( (x1_plot(i) - xobs(1, 1))^2 + (x2_plot(i) - xobs(1, 2))^2 );
    end
end
[max_cbf, val] = max( bar_f(1, :) );
max_cbf = max_cbf + 0.01;
time_align = zeros(n_sims, 1);
time = size(bar_f);
time = time(1);
for j = 1 : n_sims
    crossed = 0;
    for i = 1:time
        if bar_f(i, j) >=  max_cbf && ~crossed
            time_align(j) = i;
            bar_f(1:end - i, j) = bar_f(i:end-1, j);
            crossed = 1;
        end
    end
end


time = 0:time - 1;
time = time * dt;

font_size = 18;
figure
hold on
plot(time, bar_f, 'LineWidth', 2)
plot(time, bar_f(:, 1)*0,'--', 'LineWidth', 2)
set(gca,'FontSize', font_size)
grid 

xlim([0 20])

xlabel('$Time\,(s)$', 'Interpreter', 'latex')
ylabel('$CBF$', 'Interpreter', 'latex')
legend('$R_o=0.2m$', '$R_o=0.3m$', '$R_o=0.4$m', '$R_o=0.5m$', '$R_o=0.6m$', 'Limit', 'NumColumns', 1, 'Interpreter', 'latex', 'Location', 'eastoutside')
%%

lambda = 0:0.01:4*pi;
ang = [0:0.01:2*pi];
names = {'one_obstacle_7', 'two_obstacle', 'two_obstacle_2', 'three_obstacle'};
figure();
tiledlayout(3, 2, 'TileSpacing', 'compact'); 
for j = 1:4
    % v = VideoWriter('newfile.avi');
    % open(v)
    load(names{j})
%     subplot(2, 2, j)
    nexttile
    
    plot(r*cos(lambda), r*sin(lambda),'--r','color','green','linewidth',4);
    if mod(j, 2) ==1
       ylabel('$x_{2}(m)$', 'Interpreter', 'latex')
    end
    if j > 2
        xlabel('$x_{1}(m)$', 'Interpreter', 'latex') 
    end
    hold on
    plot(x1_plot,x2_plot,'r','color','red','linewidth',2);
    grid on;

    obs_color = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4940 0.1840 0.5560]};
    for i = 1:length(eta0)
        eta_obstacle = eta0(i);
        x0 = r*[sin(eta_obstacle), cos(eta_obstacle)];
    %     plot(x0(1)+epsilon2*cos(ang),x0(2)+epsilon2*sin(ang), 'LineWidth', 8)
        circle(x0(1), x0(2), epsilon2, obs_color{i})
    end
    k = 1;
    Frames = struct('cdata',[],'colormap',[]);
    for i = 1:T/dt
        if (mod(i,100) == 0 || i == 1)

            [x_rotated,y_rotated] = robot_display(x1_plot(i),x2_plot(i),x3_plot(i));

            %         if (i == 1)
            %             fill(x_rotated, y_rotated,'g');
            %         else
            %             h1 = plot(x_rotated, y_rotated, 'r');
            h2 = fill(x_rotated, y_rotated,'r','facealpha',0.9);
            Frames(k)=getframe(gcf);
            k = k+1;
            %             h1
            pause(eps)
            delete(h2)
            %         end
        end

        % legend({'$\chi(0)$','$\gamma$','$h(\chi)$'},'FontSize',14,'Interpreter','latex')
    end
    set(gca,'FontSize',16)
end
legend({'Desired', 'Trajectory', 'Obstacle 1', 'Obstacle 2', 'Obstacle 3'}, 'Location', 'northoutside', 'NumColumns', 4)
% savefig('samplefigure.pdf', bbox_inches='tight')
%%
names = {'one_obstacle_2', 'one_obstacle_3', 'one_obstacle_4', 'one_obstacle_5', 'one_obstacle_6', 'one_obstacle_7'};
figure();

% tiledlayout(2,2, 'TileSpacing', 'compact'); 
for j = 1:6
    hold on
    % v = VideoWriter('newfile.avi');
    % open(v)
%     load(names{j})
%     subplot(2, 2, j)
%     nexttile
    
    plot(r*cos(lambda), r*sin(lambda),'--r','color','green','linewidth',4);
    ylabel('$x_{2}(m)$', 'Interpreter', 'latex')
    xlabel('$x_{1}(m)$', 'Interpreter', 'latex') 
    plot(x1_plot,x2_plot,'r','color','red','linewidth',2);
    grid on;

    obs_color = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4940 0.1840 0.5560]};
    for i = 1:length(eta0)
        eta_obstacle = eta0(i);
        x0 = r*[sin(eta_obstacle), cos(eta_obstacle)];
    %     plot(x0(1)+epsilon2*cos(ang),x0(2)+epsilon2*sin(ang), 'LineWidth', 8)
        circle(x0(1), x0(2), epsilon2, obs_color{i})
    end
    k = 1;
    Frames = struct('cdata',[],'colormap',[]);
    for i = 1:T/dt
        if (mod(i,100) == 0 || i == 1)

            [x_rotated,y_rotated] = robot_display(x1_plot(i),x2_plot(i),x3_plot(i));

            %         if (i == 1)
            %             fill(x_rotated, y_rotated,'g');
            %         else
            %             h1 = plot(x_rotated, y_rotated, 'r');
            h2 = fill(x_rotated, y_rotated,'r','facealpha',0.9);
            Frames(k)=getframe(gcf);
            k = k+1;
            %             h1
            pause(eps)
            delete(h2)
            %         end
        end

        % legend({'$\chi(0)$','$\gamma$','$h(\chi)$'},'FontSize',14,'Interpreter','latex')
    end
    set(gca,'FontSize',16)
end
legend({'Desired', 'Trajectory', 'Obstacle 1', 'Obstacle 2', 'Obstacle 3'}, 'Location', 'northoutside', 'NumColumns', 4)
%%
names = {'one_obstacle_7'};
figure();
lambda = 0:0.01:4*pi;
ang = [0:0.01:2*pi];
% tiledlayout(2,2, 'TileSpacing', 'compact'); 
for h = 1:1
    hold on
    % v = VideoWriter('newfile.avi');
    % open(v)
%     load(names{j})
%     subplot(2, 2, j)
%     nexttile
    
    plot(r*cos(lambda), r*sin(lambda),'--r','color','green','linewidth',4);
    ylabel('$x_{2}(m)$', 'Interpreter', 'latex')
    xlabel('$x_{1}(m)$', 'Interpreter', 'latex') 
    plot(x1_plot,x2_plot,'r','color','red','linewidth',2);

    obs_color = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.4940 0.1840 0.5560]};
%     j = 1;
%     j_new = length(x1_plot);
%     q = q_store(1);
%     while j < length(x1_plot)
%         for i = j:length(x1_plot)
%             if q_store(i) ~= q
%                 j_new = i;
%                 q_new = q_store(i);
%                 break
%             end
%             j_new = i;
%             q_new = q_store(i);
%         end
%         if q == 0
%             plot(x1_plot(j:j_new), x2_plot(j:j_new), 'k', 'linewidth', 2);
%         elseif q >= 1
%             plot(x1_plot(j:j_new), x2_plot(j:j_new), 'Color', obs_color{q}, 'linewidth', 2);
%         end
%         j = j_new;
%         q = q_new;
%     end
    grid on;
    for i = 1:length(eta0)
        eta_obstacle = eta0(i);
        x0 = r*[cos(eta_obstacle), sin(eta_obstacle)];
    %     plot(x0(1)+epsilon2*cos(ang),x0(2)+epsilon2*sin(ang), 'LineWidth', 8)
        circle(x0(1), x0(2), epsilon2, obs_color{i})
    end
    k = 1;
    Frames = struct('cdata',[],'colormap',[]);
    for i = 1:T/dt
        if (mod(i,100) == 0 || i == 1)

            [x_rotated,y_rotated] = robot_display(x1_plot(i),x2_plot(i),x3_plot(i));

            %         if (i == 1)
            %             fill(x_rotated, y_rotated,'g');
            %         else
            %             h1 = plot(x_rotated, y_rotated, 'r');
            h2 = fill(x_rotated, y_rotated,'r','facealpha',0.9);
            Frames(k)=getframe(gcf);
            k = k+1;
            %             h1
            pause(eps)
            delete(h2)
            %         end
        end

        % legend({'$\chi(0)$','$\gamma$','$h(\chi)$'},'FontSize',14,'Interpreter','latex')
    end
    set(gca,'FontSize',16)
end
% legend({'Reference', 'Trajectory', 'Obstacle 1', 'Obstacle 2', 'Obstacle 3'}, 'Location', 'eastoutside', 'NumColumns', 1)