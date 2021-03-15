%% This script matches frames from different topics
%
% Authors: Manuel Simas and Bruno Guerreiro, 2021.
% Project REPLACE (for more information: http://replace.isr.tecnico.ulisboa.pt )

fprintf('Reading ROS bag topics and matching timetags (step 2/6) ... ');

%% Read all the messages in that topic as an nx1 cell array & Saving the Time Stamps
msgs_Rect_RGB = readMessages(select(bag, 'Topic', '/camera/rgb/image_rect_color'));
for i = 1:length(msgs_Rect_RGB)
    Time_Rect_RGB(i, 1) = msgs_Rect_RGB{i, 1}.Header.Stamp.Sec + msgs_Rect_RGB{i, 1}.Header.Stamp.Nsec*10^-9;
end
clear msgs_Rect_RGB

msgs_RGB_CameraInfo = readMessages(select(bag, 'Topic', '/camera/rgb/camera_info'));
for i = 1:length(msgs_RGB_CameraInfo)
    Time_RGB_CameraInfo(i, 1) = msgs_RGB_CameraInfo{i, 1}.Header.Stamp.Sec + msgs_RGB_CameraInfo{i, 1}.Header.Stamp.Nsec*10^-9;
end
clear msgs_RGB_CameraInfo

msgs_Depth = readMessages(select(bag, 'Topic', '/camera/depth/points'),'DataFormat','struct');
for i = 1:length(msgs_Depth)
    ptcloud = Struct2PointCloud(msgs_Depth{i});
    Time_Depth(i, 1) = ptcloud.Header.Stamp.Sec + ptcloud.Header.Stamp.Nsec*10^-9;
end
clear msgs_Depth

msgs_Depth_CameraInfo = readMessages(select(bag, 'Topic', '/camera/depth/camera_info'));
for i = 1:length(msgs_Depth_CameraInfo)
    Time_Depth_CameraInfo(i, 1) = msgs_Depth_CameraInfo{i, 1}.Header.Stamp.Sec + msgs_Depth_CameraInfo{i, 1}.Header.Stamp.Nsec*10^-9;
end
clear msgs_Depth_CameraInfo

msgs_Pose = readMessages(select(bag, 'Topic', '/vrpn_client_node/aero1/pose'));
for i = 1:length(msgs_Pose)
    Time_Pose(i, 1) = msgs_Pose{i, 1}.Header.Stamp.Sec + msgs_Pose{i, 1}.Header.Stamp.Nsec*10^-9;
end
clear msgs_Pose

raw_Time_Rect_RGB = Time_Rect_RGB;
raw_Time_RGB_CameraInfo = Time_RGB_CameraInfo;
raw_Time_Depth = Time_Depth;
raw_Time_Depth_CameraInfo = Time_Depth_CameraInfo;
raw_Time_Pose = Time_Pose;

%% Matching Time Stamps
for i = 1:length(Time_Rect_RGB)
   if ~ismember(Time_Rect_RGB(i), Time_Depth) || ~ismember(Time_Rect_RGB(i), Time_RGB_CameraInfo) || ~ismember(Time_Rect_RGB(i), Time_Depth_CameraInfo)
       Time_Rect_RGB(i) = 0;
   end 
end

for i = 1:length(Time_RGB_CameraInfo)
   if ~ismember(Time_RGB_CameraInfo(i), Time_Rect_RGB) || ~ismember(Time_RGB_CameraInfo(i), Time_Depth) || ~ismember(Time_RGB_CameraInfo(i), Time_Depth_CameraInfo)
       Time_RGB_CameraInfo(i) = 0;
   end
end

for i = 1:length(Time_Depth)
   if ~ismember(Time_Depth(i), Time_Rect_RGB) || ~ismember(Time_Depth(i), Time_RGB_CameraInfo) || ~ismember(Time_Depth(i), Time_Depth_CameraInfo)
       Time_Depth(i) = 0;
   end
end

for i = 1:length(Time_Depth_CameraInfo)
   if ~ismember(Time_Depth_CameraInfo(i), Time_Depth) || ~ismember(Time_Depth_CameraInfo(i), Time_Rect_RGB) || ~ismember(Time_Depth_CameraInfo(i), Time_RGB_CameraInfo)
       Time_Depth_CameraInfo(i) = 0;
   end
end

Time_Indexes_Rect_RGB = find(Time_Rect_RGB ~= 0);
Time_Indexes_Depth = find(Time_Depth ~= 0);
Time_Indexes_RGB_CameraInfo = find(Time_RGB_CameraInfo ~= 0);
Time_Indexes_Depth_CameraInfo = find(Time_Depth_CameraInfo ~= 0);

temp = nonzeros(Time_Rect_RGB);
for i = 1:length(temp)
    [Time_Indexes_Pose(i, 1), d_Pose(i, 1)] = dsearchn(Time_Pose, temp(i));
end 

%% Removing frames related to the OptiTrack failures
load(bad_frames_file);
for i = 1:length(Indexes_To_Remove)
    Time_Indexes_Rect_RGB(Indexes_To_Remove(i)) = 0;
    Time_Indexes_Depth(Indexes_To_Remove(i)) = 0;
    Time_Indexes_RGB_CameraInfo(Indexes_To_Remove(i)) = 0;
    Time_Indexes_Depth_CameraInfo(Indexes_To_Remove(i)) = 0;
    Time_Indexes_Pose(Indexes_To_Remove(i)) = 0;
end

Time_Indexes_Rect_RGB = Time_Indexes_Rect_RGB(Time_Indexes_Rect_RGB~= 0);
Time_Indexes_Depth = Time_Indexes_Depth(Time_Indexes_Depth~= 0);
Time_Indexes_RGB_CameraInfo = Time_Indexes_RGB_CameraInfo(Time_Indexes_RGB_CameraInfo~= 0);
Time_Indexes_Depth_CameraInfo = Time_Indexes_Depth_CameraInfo(Time_Indexes_Depth_CameraInfo~= 0);
Time_Indexes_Pose = Time_Indexes_Pose(Time_Indexes_Pose~= 0);

%% Saving Topics with Matched Times Stamps
msgs_Rect_RGB = readMessages(select(bag, 'Topic', '/camera/rgb/image_rect_color'));
Rect_RGB = msgs_Rect_RGB(Time_Indexes_Rect_RGB);
% save Rect_RGB -v7.3
% clear msgs_Rect_RGB

msgs_RGB_CameraInfo = readMessages(select(bag, 'Topic', '/camera/rgb/camera_info'));
RGB_CameraInfo = msgs_RGB_CameraInfo(Time_Indexes_RGB_CameraInfo);
% save RGB_CameraInfo -v7.3
% clear msgs_RGB_CameraInfo

msgs_Depth = readMessages(select(bag, 'Topic', '/camera/depth/points'),'DataFormat','struct');
Depth = msgs_Depth(Time_Indexes_Depth);
% for i=1:length(Time_Indexes_Depth)
%     Depth(i) = Struct2PointCloud(msgs_Depth{Time_Indexes_Depth(i)});
% end
% save Depth -v7.3
% clear msgs_Depth

msgs_Depth_CameraInfo = readMessages(select(bag, 'Topic', '/camera/depth/camera_info'));
Depth_CameraInfo = msgs_Depth_CameraInfo(Time_Indexes_Depth_CameraInfo);
% save Depth_CameraInfo -v7.3
% clear msgs_Depth_CameraInfo

msgs_Pose = readMessages(select(bag, 'Topic', '/vrpn_client_node/aero1/pose'));
Pose = msgs_Pose(Time_Indexes_Pose);
% save Pose -v7.3
% clear msgs_Pose

fprintf('Done\n');