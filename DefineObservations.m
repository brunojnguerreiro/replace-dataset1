%% This script prepares the filter inputs and measurements
%
% Authors: Manuel Simas and Bruno Guerreiro, 2021.
% Project REPLACE (for more information: http://replace.isr.tecnico.ulisboa.pt )

fprintf('Defining filter input and observations (step 5/6) ... ');

%% Saving the measurements
Measurements = cell(length(Depth_Points), 1);
temp = 1;

for i = 1:length(Depth_Points)
    for j = 1:size(Points_3D_Inercial{i, :}, 1)
        Measurements{i, :}(2*temp-1:2*temp, :) = [(Points_3D_Inercial{i, :}(j, 1) - Real_Trajectory(1, i)); (Points_3D_Inercial{i, :}(j, 2) - Real_Trajectory(2, i))];
        temp = temp + 1;
    end
    temp = 1;
end

%% Creating the Z Vector
len = length(Measurements);
Z = repmat({struct('NoT', {0}, 'Position_IB', {0}, 'Delta', {0}, 'Lin_Vel', {0}, 'Head', {0})}, len, 1);

for i = 1:len
    Z{i, 1}.NoT = length(Measurements{i, :})/2;
    Z{i, 1}.Position_IB = Measurements{i, :};
    Z{i, 1}.Delta = Delta(i);
    Z{i, 1}.Lin_Vel = [u(1, i); 0];
    Z{i, 1}.Head = u(2, i);
end

fprintf('Done\n');