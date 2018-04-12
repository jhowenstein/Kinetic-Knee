clear variables

filename = 'TEST_21';

DYNAMIC_CALIBRATION = true;
dt = .02; % Sampling period

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

% Extract data from main data collection file
TIME = data(1,:);
N = length(TIME);

X1 = data(4,:);
Y1 = data(3,:);
Z1 = data(2,:);

X1G = data(5,:) * RADIAN_2_DEGREE;
Y1G = data(6,:) * RADIAN_2_DEGREE;
Z1G = data(7,:) * RADIAN_2_DEGREE;

XROT = zeros(1,N);
YROT = zeros(1,N);
ZROT = zeros(1,N);

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
end

% Establish initial reference heading. In this case, the first data point
ZREF = Z1(1);

% Use reference heading to shift the heading of all following data points
for i = 1:N
    Z1(i) = shift_Z(Z1(i),ZREF);
end

% Create calibration matrix using the X and Y static orientations
alpha = Cal_X1;
beta = Cal_Y1;
gamma = 0;

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_CAL = Rz * Ry * Rx;

X1R = zeros(1,N);
Y1R = zeros(1,N);
Z1R = zeros(1,N);

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
    
    R = R_RAW * transpose(R_CAL);
    
    ALPHA_SEG = atan2d(-R(3,2),R(3,3));
    BETA_SEG = atan2d(R(3,1),sqrt((R(1,1)^2+R(2,1)^2)));
    GAMMA_SEG = atan2d(-R(2,1),R(1,1));
    
    X1R(i) = ALPHA_SEG;
    Y1R(i) = BETA_SEG;
    Z1R(i) = GAMMA_SEG;
end

% If dynamic calibration is on, load the dynamic calibration data
% headingOptimization function finds the orientation of the sensor relative
% to the plane of motion (Anterior/posterior plane)
if DYNAMIC_CALIBRATION
    dynamic_data = transpose(csvread(dynamic_file));
    X1G_DYN = dynamic_data(5,:);
    Y1G_DYN = dynamic_data(6,:);
    Z1G_DYN = dynamic_data(7,:);
    Z_OFFSET = headingOptimization(Cal_X1, Cal_Y1, X1G_DYN, Y1G_DYN, Z1G_DYN)
end

% Create another calibration using the static offset coefficients and the
% heading offset of the sensor relative to the plane of motion
alpha = Cal_X1;
beta = Cal_Y1;
gamma = 0;
if DYNAMIC_CALIBRATION
    gamma = Z_OFFSET;
end

Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

R_CAL = Rz * Ry * Rx;

X1GR = zeros(1,N);
Y1GR = zeros(1,N);
Z1GR = zeros(1,N); 

for i = 2:N
    alpha = 0;
    beta = YROT(i-1);
    gamma = ZROT(i-1);

    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

    R_CURR = Rz * Ry * Rx;
        
    GYRO = [X1G(i); Y1G(i); Z1G(i)];
    % Rotate the gyroscope values based on the calibration matrix
    GYRO_R = transpose(R_CAL) * GYRO;
    % Rotate gyroscope values based on current orientation of the sensor
    GYRO_R = transpose(R_CURR) * GYRO_R;
        
    X1GR(i) = GYRO_R(1,1);
    Y1GR(i) = GYRO_R(2,1);
    Z1GR(i) = GYRO_R(3,1);
    
    % Integrate the gyro values to calculate relative motion
    XROT(i) = XROT(i-1) + (X1GR(i) * dt);
    YROT(i) = YROT(i-1) + (Y1GR(i) * dt);
    ZROT(i) = ZROT(i-1) + (Z1GR(i) * dt); 
end

figure(1)
plot(TIME,X1,TIME,Y1,TIME,Z1)
title('Sensor Orientations')
legend('X','Y','Z')

figure(2)
plot(TIME,XROT,TIME,YROT,TIME,ZROT)
title('Gyro Derived Orientations')
legend('X','Y','Z')

figure(3)
plot(TIME,X1G,TIME,Y1G,TIME,Z1G)
title('Original Gyro')
legend('X','Y','Z')

figure(4)
plot(TIME,X1GR,TIME,Y1GR,TIME,Z1GR)
title('Rotated Gyro')
legend('X','Y','Z')

figure(5)
subplot(3,1,1)
plot(TIME,X1G,TIME,X1GR)
title('X Gyro')
legend('Original','Rotated')
subplot(3,1,2)
plot(TIME,Y1G,TIME,Y1GR)
title('Y Gyro')
legend('Original','Rotated')
subplot(3,1,3)
plot(TIME,Z1G,TIME,Z1GR)
title('Z Gyro')
legend('Original','Rotated')

figure(6)
subplot(3,1,1)
plot(TIME,XROT)
title('X ROTATION')
subplot(3,1,2)
plot(TIME,YROT)
title('Y ROTATION')
subplot(3,1,3)
plot(TIME,ZROT)
title('Z ROTATION')

figure(7)
subplot(3,1,1)
plot(TIME,X1R,TIME,XROT)
title('X ROTATION')
legend('Original','Gyro')
subplot(3,1,2)
plot(TIME,Y1R,TIME,YROT)
title('Y ROTATION')
legend('Original','Gyro')
subplot(3,1,3)
plot(TIME,Z1R,TIME,ZROT)
title('Z ROTATION')
legend('Original','Gyro')


