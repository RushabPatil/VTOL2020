function v_rot = midterm_rot(v, gamma, psi, phi, Transpose)

% Convert from I to E via I-A, A-B, B-E in a 2-3-1 euler angle scheme
% E: body-fixed frame; I: Inertial frame

% Inputs:
% v: Initial vector
% Define angles (counter clockwise is greater than 0)
% phi: Angle of x-rotation (roll)
% psi: Angle of y-rotation (heading)
% gamma: Angle of z-rotation (angle of climb)
% Note that angles are in radians 
% Transpose: Directional indicator (0: convert I -> E; 1: convert E -> I)

% Create rotation matrices
z_rot = [cos(gamma) sin(gamma) 0; -sin(gamma) cos(gamma) 0; 0 0 1];
y_rot = [cos(psi) 0 -sin(psi); 0 1 0; sin(psi) 0 cos(psi)];
x_rot = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];

if Transpose == 0   % Rotating from I to E
    rot = x_rot * z_rot * y_rot;
else                % Rotating from E to I
    rot = y_rot' * z_rot' * x_rot';
end
v_rot = rot * v;