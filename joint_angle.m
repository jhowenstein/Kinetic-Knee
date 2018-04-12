function [ ALPHA_JOINT, BETA_JOINT, GAMMA_JOINT ] = joint_angle( X1, Y1, Z1, X2, Y2, Z2 )

% Proximal Segment
alpha = X1;
beta = Y1;
gamma = Z1;

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_PROX = Rz * Ry * Rx;

% Distal Segment
alpha = X2;
beta = Y2;
gamma = Z2;

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_DIST = Rz * Ry * Rx;

% Knee Decomposition

R_JOINT = R_DIST * transpose(R_PROX);

ALPHA_JOINT = atan2d(-R_JOINT(3,2),R_JOINT(3,3));
BETA_JOINT = atan2d(R_JOINT(3,1),sqrt((R_JOINT(1,1)^2+R_JOINT(2,1)^2)));
GAMMA_JOINT = atan2d(-R_JOINT(2,1),R_JOINT(1,1));


end

