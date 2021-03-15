%% This script plots some variables from the dataset
%
% Authors: Manuel Simas and Bruno Guerreiro, 2021.
% Project REPLACE (for more information: http://replace.isr.tecnico.ulisboa.pt )

fprintf('Ploting some data ... ');

%% Show times for different sources
figure(1);
t_pose = Time_Pose(Time_Pose~=0);
t_rgb = Time_Rect_RGB(Time_Rect_RGB~=0);
t_rgbinfo = Time_RGB_CameraInfo(Time_RGB_CameraInfo~=0);
t_depth = Time_Depth(Time_Depth~=0);
t_depthinfo = Time_Depth_CameraInfo(Time_Depth_CameraInfo~=0);
t_ini = t_pose(1);
t_end = t_pose(end);
subplot(121);
plot(t_pose-t_ini);
hold on;
plot(t_rgb-t_ini);
plot(t_rgbinfo-t_ini);
plot(t_depth-t_ini);
plot(t_depthinfo-t_ini);
hold off;
xlabel('Sample');
ylabel('Time [s]');
legend('Pose','RGB','RGB Info','Depth','Depth Info');
title(['Time (t_{ini} = ' num2str(t_ini) ')']);

subplot(122);
dt_pose = t_pose(2:end) - t_pose(1:end-1);
dt_rgb = t_rgb(2:end) - t_rgb(1:end-1);
dt_rgbinfo = t_rgbinfo(2:end) - t_rgbinfo(1:end-1);
dt_depth = t_depth(2:end) - t_depth(1:end-1);
dt_depthinfo = t_depthinfo(2:end) - t_depthinfo(1:end-1);
plot(dt_pose);
hold on;
plot(dt_rgb);
plot(dt_rgbinfo);
plot(dt_depth);
plot(dt_depthinfo);
hold off;
xlabel('Sample');
ylabel('\Delta Time [s]');
legend('Pose','RGB','RGB Info','Depth','Depth Info');
title('Time diferences');


%% Showing the images, point cloud and inertial trajectory and features
figure(2);
ax_max = [0 1 0 1 0 1];
objects = [];
Nsteps = 100;
N = length(Rect_RGB);
for i = 1:round(N/Nsteps):N
    subplot(131)
    img = readImage(Rect_RGB{i});
    imshow(img);
    title(['RGB image (t = ' num2str(Time_Rect_RGB(i)-t_ini) '/' num2str(t_end-t_ini) ' s)']);
    
    subplot(132)
    Rot_Matrix = [1   0   0; 0   0   1; 0  -1   0];
    pts = (Rot_Matrix*Depth_Points{i}.Position')';
    if ~isempty(pts)
        plot3(0,0,0,'+g');
        hold on;
        plot3(pts(:,1),pts(:,2),pts(:,3),'.b');
        if ~isempty(Points_3D_Body{i})
            plot3(Points_3D_Body{i}(:,1),Points_3D_Body{i}(:,2),Points_3D_Body{i}(:,3),'xr');
        end
        grid on;
        ax = axis;
        ax_max([1,3,5]) = floor(min(ax_max([1,3,5]),ax([1,3,5])));
        ax_max([2,4,6]) = ceil(max(ax_max([2,4,6]),ax([2,4,6])));
        ax_max([1,3,5]) = ceil(max([-5,-5,-5],ax_max([1,3,5])));
        ax_max([2,4,6]) = floor(min([5,5,5],ax_max([2,4,6])));
        axis equal;
        axis(ax_max);
        hold off;
        xlabel('x [m]', 'Interpreter', 'latex', 'FontSize', 15);
        ylabel('y [m]', 'Interpreter', 'latex', 'FontSize', 15);
        zlabel('z [m]', 'Interpreter', 'latex', 'FontSize', 15);
        title(['Body depth point cloud (t = ' num2str(Time_Rect_RGB(i)-t_ini) '/' num2str(t_end-t_ini) ' s)']);
    end
    
    subplot(133);
    Traj = plot3(Real_Trajectory(1, 1:i), Real_Trajectory(2, 1:i), Real_Trajectory(3, 1:i), '.-', 'Color', 'b');
    hold on; 
    Init = plot3(Real_Trajectory(1, 1), Real_Trajectory(2, 1), Real_Trajectory(3, 1), 'd', 'Color', 'g');
    % plot landmarks:
    if ~isempty(Points_3D_Inercial{i, :})
        objects = [objects , Points_3D_Inercial{i, :}'];
    end
    if ~isempty(objects)
        Objs = plot3(objects(1, :),objects(2, :),objects(3, :),'+','Color','m');
        %legend([Init Traj Objs], {'Initial Position','OptiTrack','Objects'});
    end
    % Plot qoadcopter at current position
    QuadPlot(Real_Trajectory(:,i),R2Euler(RTT{i}),0);
    hold off;
    axis equal;
    grid on;
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    zlabel('z [m]', 'Interpreter', 'latex');
%     set(gca,'Ydir','reverse');
%     set(gca,'Zdir','reverse');
    title(['Inertial trajectory and features (t = ' num2str(Time_Rect_RGB(i)-t_ini) '/' num2str(t_end-t_ini) ' s)']);
    
    pause(0.05);
end

fprintf('Done\n');