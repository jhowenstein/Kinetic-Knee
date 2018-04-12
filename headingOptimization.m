function [ Z_OFFSET ] = headingOptimization( Cal_X1, Cal_Y1, X1G, Y1G, Z1G )

N = length(X1G);
X1GR = zeros(1,N);
Y1GR = zeros(1,N);
Z1GR = zeros(1,N);

min_lateral_val = 999;
for k = 0:45
    alpha = Cal_X1;
    beta = Cal_Y1;
    gamma = k;

    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

    R_CAL = Rz * Ry * Rx;

    for i = 1:N
        GYRO = [X1G(i); Y1G(i); Z1G(i)];
        GYRO_R = transpose(R_CAL) * GYRO;

        X1GR(i) = GYRO_R(1,1);
        Y1GR(i) = GYRO_R(2,1);
        Z1GR(i) = GYRO_R(3,1);  
    end
    
    X_MEAN = mean(abs(X1GR));
    Y_MEAN = mean(abs(Y1GR));
    Z_MEAN = mean(abs(Z1GR));
    
    sum_lateral = Y_MEAN + Z_MEAN;
    
    if sum_lateral < min_lateral_val
        min_lateral_val = sum_lateral;
        Z_OFFSET = k;
    end
end

for k = -1:-1:-45
    alpha = Cal_X1;
    beta = Cal_Y1;
    gamma = k;

    Rx = [1, 0, 0; 0, cosd(alpha), sind(alpha); 0, -sind(alpha), cosd(alpha)];
    Ry = [cosd(beta), 0, -sind(beta); 0, 1, 0; sind(beta), 0, cosd(beta)];
    Rz = [cosd(gamma), sind(gamma), 0; -sind(gamma), cosd(gamma), 0; 0, 0, 1];

    R_CAL = Rz * Ry * Rx;

    for i = 1:N
        GYRO = [X1G(i); Y1G(i); Z1G(i)];
        GYRO_R = transpose(R_CAL) * GYRO;

        X1GR(i) = GYRO_R(1,1);
        Y1GR(i) = GYRO_R(2,1);
        Z1GR(i) = GYRO_R(3,1);  
    end
    
    X_MEAN = mean(abs(X1GR));
    Y_MEAN = mean(abs(Y1GR));
    Z_MEAN = mean(abs(Z1GR));
    
    sum_lateral = Y_MEAN + Z_MEAN;
    
    if sum_lateral < min_lateral_val
        min_lateral_val = sum_lateral;
        Z_OFFSET = k;
    end
end


end

