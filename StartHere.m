%% This script processes all data from the original ROSBAG file, preparing a
% smaller file as a Matlab workspace to be used by the proposed algorithm.
%
% Authors: Manuel Simas and Bruno Guerreiro, 2021.
% Project REPLACE (for more information: http://replace.isr.tecnico.ulisboa.pt )

clear all; close all

raw_data_file = '2019-07-10_SLAMMOT_Alameda.bag';
bad_frames_file = '2019-07-10_SLAMMOT_Alameda_BadFrames.mat';

processed_data_file = [raw_data_file(1:end-4) '_Observations.mat'];

if ~exist(processed_data_file,'file')

    % Retrieve from web (if not already done) and open ROSbag file:
    fprintf('Getting/opening raw data file (may download ~8GB from web - step 1/6) ... ');
    if ~exist(raw_data_file,'file')
        % ROSBAG file also available at: https://drive.google.com/file/d/1HiKYGGFMMEVFt40XkEVDAvK1GRC4JLGB/view?usp=sharing
        url = 'http://users.isr.ist.utl.pt/~bguerreiro/rosbags/2019-07-10_SLAMMOT_Alameda.bag';
        websave(raw_data_file,url);
    end
    bag = rosbag(raw_data_file);
    bagInfo = rosbag('info',raw_data_file);
    fprintf('Done\n');

    MatchFrames;        % match timetamps and remove bad mocap data
    GetFeatures;        % detect features and convert them to body coordinates
    GetMotion;          % get vehicle motion data
    DefineObservations; % generate observation and input vector

    fprintf('Saving processed data (step 6/6)... ');
    save(processed_data_file,'Z','Real_Trajectory','RTT','bagInfo',...
        'Time_Pose','Time_Rect_RGB','Time_RGB_CameraInfo','Time_Depth','Time_Depth_CameraInfo',...
        'Rect_RGB','Depth_Points','Points_3D_Body','Points_3D_Inercial','-v7.3');
    fprintf('Done\n');
    
else % just plot the processed data
    fprintf('Loading previously processed data ... ');
    load(processed_data_file);
    fprintf('Done\n');
end

% plots basic data from the rosbag
ShowData;

