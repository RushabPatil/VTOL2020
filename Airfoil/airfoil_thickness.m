function total_height = airfoil_thickness(air_foil_input)
% air_foil_input: the file name containing the points comprising the
% airfoil geometry (excluding the .txt). To ensure commonality with the
% input required for air_foil_find.m, ensure the .txt files have the same
% name

% This function will only work if the data is points are arranged in a
% n-by-2 matrix, where column 1 is horizontal position and column 2 is the
% vertical position. There must also not be the airfoil name in the first
% row (which is the case in .dat files for airfoils)

% format the filename
string = join([air_foil_input,'.txt']);

% open the .txt file
fileID = fopen(string,'r');

% convert the contents of the .txt file to an Nx2 matrix
formatSpec = '%f %f';
size_points = [2 Inf];
points = fscanf(fileID,formatSpec,size_points)';

% find the extrema along the y-axis of the airfoil
Max = max(points(:,2));
Min = min(points(:,2));

% calculate the difference between the extrema
total_height = Max - Min;
