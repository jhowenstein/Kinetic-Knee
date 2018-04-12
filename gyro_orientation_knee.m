clear variables

filename = 'squat5';

DYNAMIC_CALIBRATION = true;
dt = .02;

RADIAN_2_DEGREE = 57.3; % Conversion factor for radians to degrees
GYRO_CUTOFF = 5;

data_file = strcat(filename,'.csv');
cal_file = strcat(filename,'_cal.csv');
coeff_file = strcat(filename,'_coeff.csv');
dynamic_file = strcat(filename,'_dynamic.csv');

% Load data from data file, static calibration, and dynamic calibration
data = transpose(csvread(data_file));
cal_data = transpose(csvread(cal_file));
cal_coeff = transpose(csvread(coeff_file));

% Extract static calibration coefficients
Cal_X1 = cal_coeff(4);
Cal_Y1 = cal_coeff(3);
Cal_Z1 = cal_coeff(2);

Cal_X2 = cal_coeff(10);
Cal_Y2 = cal_coeff(9);
Cal_Z2 = cal_coeff(8);

% Extract data from main data collection file
TIME = data(1,:);
N = length(TIME);

X1 = data(4,:);
Y1 = data(3,:);
Z1 = data(2,:);

X1G = data(5,:) * RADIAN_2_DEGREE;
Y1G = data(6,:) * RADIAN_2_DEGREE;
Z1G = data(7,:) * RADIAN_2_DEGREE;

X2 = data(10,:);
Y2 = data(9,:);
Z2 = data(8,:);

X2G = data(11,:) * RADIAN_2_DEGREE;
Y2G = data(12,:) * RADIAN_2_DEGREE;
Z2G = data(13,:) * RADIAN_2_DEGREE;

X1ROT = zeros(1,N);
Y1ROT = zeros(1,N);
Z1ROT = zeros(1,N);

X2ROT = zeros(1,N);
Y2ROT = zeros(1,N);
Z2ROT = zeros(1,N);

for i = 1:N
    if abs(X1G(i)) < GYRO_CUTOFF
        X1G(i) = 0;
    end
    if abs(Y1G(i)) < GYRO_CUTOFF
        Y1G(i) = 0;
    end
    if abs(Z1G(i)) < GYRO_CUTOFF
        Z1G(i) = 0;
    end
    
    if abs(X2G(i)) < GYRO_CUTOFF
        X2G(i) = 0;
    end
    if abs(Y2G(i)) < GYRO_CUTOFF
        Y2G(i) = 0;
    end
    if abs(Z2G(i)) < GYRO_CUTOFF
        Z2G(i) = 0;
    end
end

% Establish initial reference heading. In this case, the first data point
Z1REF = Z1(1);
Z2REF = Z2(1);

% Use reference heading to shift the heading of all following data points
for i = 1:N
    Z1(i) = shift_Z(Z1(i),Z1REF);
    Z2(i) = shift_Z(Z2(i),Z2REF);
end

% Create sensor 1 calibration matrix using the X and Y static orientations
alpha = Cal_X1;
beta = Cal_Y1;
gamma = 0;

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_CAL1 = Rz * Ry * Rx;

% Create sensor 2 calibration matrix using the X and Y static orientations
alpha = Cal_X2;
beta = Cal_Y2;
gamma = 0;

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_CAL2 = Rz * Ry * Rx;

X1R = zeros(1,N);
Y1R = zeros(1,N);
Z1R = zeros(1,N);

X2R = zeros(1,N);
Y2R = zeros(1,N);
Z2R = zeros(1,N);

% Rotate orientation values for all data points based on calibration matrix
% Highly useful for calculating correct X orientation 
for i = 1:N
    alpha = X1(i);
    beta = Y1(i);
    gamma = Z1(i);
    
    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];
    
    R_RAW = Rz * Ry * Rx;
    
    R = R_RAW * transpose(R_CAL1);
    
    ALPHA_SEG = atan2d(-R(3,2),R(3,3));
    BETA_SEG = atan2d(R(3,1),sqrt((R(1,1)^2+R(2,1)^2)));
    GAMMA_SEG = atan2d(-R(2,1),R(1,1));
    
    X1R(i) = ALPHA_SEG;
    Y1R(i) = BETA_SEG;
    Z1R(i) = GAMMA_SEG;
    
    alpha = X2(i);
    beta = Y2(i);
    gamma = Z2(i);
    
    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];
    
    R_RAW = Rz * Ry * Rx;
    
    R = R_RAW * transpose(R_CAL2);
    
    ALPHA_SEG = atan2d(-R(3,2),R(3,3));
    BETA_SEG = atan2d(R(3,1),sqrt((R(1,1)^2+R(2,1)^2)));
    GAMMA_SEG = atan2d(-R(2,1),R(1,1));
    
    X2R(i) = ALPHA_SEG;
    Y2R(i) = BETA_SEG;
    Z2R(i) = GAMMA_SEG;
end

% If dynamic calibration is on, load the dynamic calibration data
% headingOptimization function finds the orientation of the sensor relative
% to the plane of motion (Anterior/posterior plane)
if DYNAMIC_CALIBRATION
    dynamic_data = transpose(csvread(dynamic_file));
    X1G_DYN = dynamic_data(5,:);
    Y1G_DYN = dynamic_data(6,:);
    Z1G_DYN = dynamic_data(7,:);
    
    X2G_DYN = dynamic_data(11,:);
    Y2G_DYN = dynamic_data(12,:);
    Z2G_DYN = dynamic_data(13,:);
    
    Z1_OFFSET = headingOptimization(Cal_X1, Cal_Y1, X1G_DYN, Y1G_DYN, Z1G_DYN)
    Z2_OFFSET = headingOptimization(Cal_X2, Cal_Y2, X2G_DYN, Y2G_DYN, Z2G_DYN)
end

% Create another calibration using the static offset coefficients and the
% heading offset of sensor 1 relative to the plane of motion
alpha = Cal_X1;
beta = Cal_Y1;
gamma = 0;
if DYNAMIC_CALIBRATION
    gamma = Z1_OFFSET;
end

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_CAL1 = Rz * Ry * Rx;

% Create another calibration using the static offset coefficients and the
% heading offset of sensor 2 relative to the plane of motion
alpha = Cal_X2;
beta = Cal_Y2;
gamma = 0;
if DYNAMIC_CALIBRATION
    gamma = Z2_OFFSET;
end

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_CAL2 = Rz * Ry * Rx;

X1GR = zeros(1,N);
Y1GR = zeros(1,N);
Z1GR = zeros(1,N); 

X2GR = zeros(1,N);
Y2GR = zeros(1,N);
Z2GR = zeros(1,N); 

for i = 2:N
    alpha = 0;
    beta = Y1ROT(i-1);
    gamma = Z1ROT(i-1);
    
    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];
    
    R_CURR = Rz * Ry * Rx;
    
    GYRO = [X1G(i); Y1G(i); Z1G(i)];
    % Rotate the gyroscope values based on the calibration matrix
    GYRO_R = transpose(R_CAL1) * GYRO;
    % Rotate gyroscope values based on current orientation of the sensor
    GYRO_R = transpose(R_CURR) * GYRO_R;
    
    X1GR(i) = GYRO_R(1,1);
    Y1GR(i) = GYRO_R(2,1);
    Z1GR(i) = GYRO_R(3,1);  

    % Integrate the gyro values to calculate relative motion
    X1ROT(i) = X1ROT(i-1) + (X1GR(i) * dt);
    Y1ROT(i) = Y1ROT(i-1) + (Y1GR(i) * dt);
    Z1ROT(i) = Z1ROT(i-1) + (Z1GR(i) * dt);
    
    alpha = 0;
    beta = Y2ROT(i-1);
    gamma = Z2ROT(i-1);
    
    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];
    
    R_CURR = Rz * Ry * Rx;
    
    GYRO = [X2G(i); Y2G(i); Z2G(i)];
    % Rotate the gyroscope values based on the calibration matrix
    GYRO_R = transpose(R_CAL2) * GYRO;
    % Rotate gyroscope values based on current orientation of the sensor
    GYRO_R = transpose(R_CURR) * GYRO_R;
    
    X2GR(i) = GYRO_R(1,1);
    Y2GR(i) = GYRO_R(2,1);
    Z2GR(i) = GYRO_R(3,1);  

    % Integrate the gyro values to calculate relative motion
    X2ROT(i) = X2ROT(i-1) + (X2GR(i) * dt);
    Y2ROT(i) = Y2ROT(i-1) + (Y2GR(i) * dt);
    Z2ROT(i) = Z2ROT(i-1) + (Z2GR(i) * dt);
end

hip_x = zeros(1,N);
hip_y = zeros(1,N);
hip_z = zeros(1,N);
knee_x = zeros(1,N);
knee_y = zeros(1,N);
knee_z = zeros(1,N);
ankle_x = zeros(1,N);
ankle_y = zeros(1,N);
ankle_z = zeros(1,N);

for i = 1:N
    [hip_x(i), hip_y(i), hip_z(i)] = joint_angle(0,0,0,X1ROT(i),Y1ROT(i),Z1ROT(i));
    [knee_x(i), knee_y(i), knee_z(i)] = joint_angle(X1ROT(i),Y1ROT(i),Z1ROT(i),X2ROT(i),Y2ROT(i),Z2ROT(i));
    [ankle_x(i), ankle_y(i), ankle_z(i)] = joint_angle(X2ROT(i), Y2ROT(i), Z2ROT(i), 0, 0, 0);
end

figure(1)
plot(TIME,X1R,TIME,Y1R,TIME,Z1R)
title('Sensor 1: Rotated Orientations')
legend('X','Y','Z')

figure(2)
plot(TIME,X2R,TIME,Y2R,TIME,Z2R)
title('Sensor 2: Rotated Orientations')
legend('X','Y','Z')

figure(3)
plot(TIME,X1GR,TIME,Y1GR,TIME,Z1GR)
title('Sensor 1: Rotated Gyro')
legend('X','Y','Z')

figure(4)
plot(TIME,X2GR,TIME,Y2GR,TIME,Z2GR)
title('Sensor 2: Rotated Gyro')
legend('X','Y','Z')

figure(5)
plot(TIME,X1ROT,TIME,Y1ROT,TIME,Z1ROT)
title('Sensor 1: Gyro Derived Orientations')
legend('X','Y','Z')

figure(6)
plot(TIME,X2ROT,TIME,Y2ROT,TIME,Z2ROT)
title('Sensor 2: Gyro Derived Orientations')
legend('X','Y','Z')

figure(7)
subplot(3,1,1)
plot(TIME,X1,TIME,X1ROT)
title('Sensor 1: X ROTATION')
legend('Original','Gyro')
subplot(3,1,2)
plot(TIME,Y1,TIME,Y1ROT)
title('Sensor 1: Y ROTATION')
legend('Original','Gyro')
subplot(3,1,3)
plot(TIME,Z1,TIME,Z1ROT)
title('Sensor 1: Z ROTATION')
legend('Original','Gyro')

figure(8)
subplot(3,1,1)
plot(TIME,X2,TIME,X2ROT)
title('Sensor 2: X ROTATION')
legend('Original','Gyro')
subplot(3,1,2)
plot(TIME,Y2,TIME,Y2ROT)
title('Sensor 2: Y ROTATION')
legend('Original','Gyro')
subplot(3,1,3)
plot(TIME,Z2,TIME,Z2ROT)
title('Sensor 2: Z ROTATION')
legend('Original','Gyro')

figure(9)
subplot(3,1,1)
plot(TIME,X1ROT)
title('Thigh Orientation: X')
subplot(3,1,2)
plot(TIME,Y1ROT)
title('Thigh Orientation: Y')
subplot(3,1,3)
plot(TIME,Z1ROT)
title('Thigh Orientation: Z')

figure(10)
subplot(3,1,1)
plot(TIME,X2ROT)
title('Shank Orientation: X')
subplot(3,1,2)
plot(TIME,Y2ROT)
title('Shank Orientation: Y')
subplot(3,1,3)
plot(TIME,Z2ROT)
title('Shank Orientation: Z')

figure(11)
subplot(3,1,1)
plot(TIME,ankle_x)
title('Ankle Flexion')
subplot(3,1,2)
plot(TIME,ankle_y)
title('Ankle (Ab/Ad)duction')
subplot(3,1,3)
plot(TIME,ankle_z)
title('Ankle Internal/External Rotation')

figure(12)
subplot(3,1,1)
plot(TIME,hip_x)
title('Hip Flexion')
subplot(3,1,2)
plot(TIME,hip_y)
title('Hip (Ab/Ad)duction')
subplot(3,1,3)
plot(TIME,hip_z)
title('Hip Internal/External Rotation')

figure(13)
subplot(3,1,1)
plot(TIME,knee_x)
title('Knee Flexion')
subplot(3,1,2)
plot(TIME,knee_y)
title('Knee (Ab/Ad)duction')
subplot(3,1,3)
plot(TIME,knee_z)
title('Knee Internal/External Rotation')

