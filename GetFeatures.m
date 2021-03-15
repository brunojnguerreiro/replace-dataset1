%% Features Extraction
%
% Authors: Manuel Simas and Bruno Guerreiro, 2021.
% Project REPLACE (for more information: http://replace.isr.tecnico.ulisboa.pt )

fprintf('Detecting objects on image and depth data (step 3/6) ... ');

Features = repmat({struct('TimeStamp', {{struct('Sec', {0}, 'Nsec', {0})}}, 'Position', {0})}, length(Rect_RGB), 1);

for i = 1:length(Rect_RGB)
    img = readImage(Rect_RGB{i});
    
    Thr = 0.02;
    
    % Test All
    [temp_Feat, ROIs] = FeaturesDetectionDiffROIs(img, 4, 2, 3, Thr);    
   
    temp = 1;
    for j = 1:length(temp_Feat)
        if ~isempty(temp_Feat{j})
            Features{i, 1}.Position(temp, 1:2) = temp_Feat{j}.Location;
            temp = temp + 1;
        end
    end
    
    Features{i, 1}.TimeStamp{1, 1}.Sec = Rect_RGB{i}.Header.Stamp.Sec;
    Features{i, 1}.TimeStamp{1, 1}.Nsec = Rect_RGB{i}.Header.Stamp.Nsec;
end

%% Depth Points Extraction
Depth_Points = repmat({struct('TimeStamp', {{struct('Sec', {0}, 'Nsec', {0})}}, 'Position', {0})}, length(Depth), 1);

for i = 1:length(Depth)
        ptcloud = Struct2PointCloud(Depth{i});
%         ptcloud = Depth{i};
        xyz = readXYZ(ptcloud);
        xyz_clean = xyz(all(~isnan(xyz),2), :);

        Depth_Points{i, 1}.Position = xyz_clean;
        Depth_Points{i, 1}.TimeStamp{1, 1}.Sec = Depth{i, 1}.Header.Stamp.Sec;
        Depth_Points{i, 1}.TimeStamp{1, 1}.Nsec = Depth{i, 1}.Header.Stamp.Nsec;
end

%% RGB Camera Matrix

for i = 1:length(RGB_CameraInfo)
    P_RGB{i, 1}(1, 1:4) = RGB_CameraInfo{i, 1}.P(1:4);
    P_RGB{i, 1}(2, 1:4) = RGB_CameraInfo{i, 1}.P(5:8);
    P_RGB{i, 1}(3, 1:4) = RGB_CameraInfo{i, 1}.P(9:12);
end

%% Convert 3D Points to 2D

for i = 1:length(Depth_Points)
    for j = 1:length(Depth_Points{i, 1}.Position)
        temp = [Depth_Points{i, 1}.Position(j, :) 1];
        temp_Point = P_RGB{i}*temp';
        Points_2D{i}(j, 1) = temp_Point(1)/temp_Point(3);
        Points_2D{i}(j, 2) = temp_Point(2)/temp_Point(3);
    end    
end

%% Match 2D Depth Points with Features
k = cell(1, length(Depth_Points));
d = cell(1, length(Depth_Points));

for i = 1:length(Depth_Points)
    if ~isequal(Features{i, 1}.Position, 0)
        X = Points_2D{i};
        XI = Features{i, 1}.Position;
        [k{i}, d{i}] = dsearchn(X, XI);
    end
end    

%% Save 3D Points from the Match
Points_3D = cell(length(Depth_Points), 1);
temp = 1;

for i = 1:length(Depth_Points)
    for j = 1:length(k{i})
        if d{i}(j) < 1.5
            Points_3D{i, :}(temp, :) = Depth_Points{i, 1}.Position(k{i}(j), :);
            Used_2D{i, :}(temp, :) = Points_2D{i}(k{i}(j), :);
            k_used{i}(:, temp) = k{i}(j);
            d_used{i}(:, temp) = d{i}(j);
            temp = temp + 1;            
        end
    end
    temp = 1;
end

%% Converting 3D points from Camera Coordinate System (CS) to the Body CS
Points_3D_Body = cell(length(Depth_Points), 1);
Rot_Matrix = [1   0   0
              0   0   1
              0  -1   0];

for i = 1:length(Depth_Points)
    for j = 1:size(Points_3D{i, :}, 1)
        Points_3D_Body{i, :}(j, :) = Rot_Matrix*Points_3D{i, :}(j, :)';
    end
end

fprintf('Done\n');