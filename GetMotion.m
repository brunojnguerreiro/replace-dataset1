%% This script gets vehicle motion variables from topics
%
% Authors: Manuel Simas and Bruno Guerreiro, 2021.
% Project REPLACE (for more information: http://replace.isr.tecnico.ulisboa.pt )

fprintf('Getting vehicle motion from groundtruth data (step 4/6) ... ');

%% Pose Orientation Vector (Quarternion)
RTT = cell(length(Depth_Points), 1);

for i = 1:length(Pose) 
    qw = Pose{i, 1}.Pose.Orientation.W;
    qx = Pose{i, 1}.Pose.Orientation.X;
    qy = Pose{i, 1}.Pose.Orientation.Y;
    qz = Pose{i, 1}.Pose.Orientation.Z;

    RTT{i} = [1-2*(qy^2 + qz^2)   2*(qx*qy - qz*qw) 2*(qx*qz+qy*qw);
           2*(qx*qy+qz*qw)    1-2*(qx^2 + qz^2) 2*(qy*qz - qx*qw);
           2*(qx*qz - qy*qw)   2*(qy*qz - qx*qw) 1-2*(qx^2 + qy^2)];
end

%% Converting 3D points from Body CS to the Inercial CS
Points_3D_Inercial = cell(length(Depth_Points), 1);

for i = 1:length(Pose)
    Real_Trajectory(:, i) = [Pose{i, 1}.Pose.Position.X Pose{i, 1}.Pose.Position.Y Pose{i, 1}.Pose.Position.Z]; 
end

for i = 1:length(Depth_Points)
    for j = 1:size(Points_3D{i, :}, 1)        
        Points_3D_Inercial{i, :}(j, :) = (RTT{i}*Points_3D_Body{i, :}(j, :)' + Real_Trajectory(:, i))';
    end
end

%% Save Velocity, Heading and Delta from Pose
for i = 2:length(Pose)
    time_1 = Pose{i-1, 1}.Header.Stamp.Sec + Pose{i-1, 1}.Header.Stamp.Nsec*10^-9;  
    time = Pose{i, 1}.Header.Stamp.Sec + Pose{i, 1}.Header.Stamp.Nsec*10^-9; 
    vx = (Pose{i, 1}.Pose.Position.X - Pose{i-1, 1}.Pose.Position.X)/(time - time_1);
    vy = (Pose{i, 1}.Pose.Position.Y - Pose{i-1, 1}.Pose.Position.Y)/(time - time_1);
    
    Velocity(i) = sqrt(vx^2 + vy^2);
    Heading(i) = atan2(vy, vx);
    Delta(i) = (time - time_1);
end

%% Creating the U Vector
% U Vector - Real Values.
for i = 1:length(Velocity)
    u(1, i) = Velocity(i);
    u(2, i) = Heading(i);
end

% U Vector - Noisy Values.
for i = 1:length(Velocity)
    u(1, i) = Velocity(i) + random('norm', 0, 0.1);
    u(2, i) = Heading(i) + random('norm', 0, 10*(pi/180));
end

fprintf('Done\n');
