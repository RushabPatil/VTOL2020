% Aerospace MQP VTOL UAV Optimizer motor_find
function [motor_info, motor_mass] = motor_find(motor_input);
string = motor_input + '.txt';

% open the corresponding .txt file
fileID = fopen(string,'r');

% convert the contents of the .txt file to an Nx2 matrix
formatSpec = '%f %f %f %f %f %f';
size_points = [5 Inf];
motor_info_interim = fscanf(fileID,formatSpec,size_points)';
% leave out the last column as this only contains repeated referencess to
% the motor mass

g = 9.81;   % gravity, m s^-2
% columns of useful stuff: [power, thrust(N)], attaching [0, 0] to the top
motor_info = motor_info_interim(:,1:end-1);
motor_mass = motor_info_interim(1,end);
end