function bias = estimate_imu_bias(imu_data)
% estimate_imu_bias - Computes the average bias from stationary IMU readings
%
% Input:
%   imu_data - Nx3 matrix of IMU readings [x, y, z] over time
%              (e.g., accelerometer or gyroscope data)
%
% Output:
%   bias     - 1x3 vector containing bias for x, y, z axes

    % Ensure input is valid
    if size(imu_data, 2) ~= 3
        error('Input must be an N×3 matrix of IMU readings (x, y, z).');
    end

    % Compute the mean (stationary bias)
    bias = mean(imu_data, 1);  % 1x3 vector

end

function out = kbhit()
    out = ~isempty(get(gcf, 'CurrentCharacter'));
    if out
        set(gcf, 'CurrentCharacter', char(0));  % reset
    end
end

function [scale_matrix, bias, noise_var_matrix, apply_correction] = calibrate_imu(raw_measurements, expected_vector)
% calibrate_imu - Estimates bias, noise variance, and provides a correction function for IMU data.
%
% Inputs:
%   raw_measurements   : Nx3 matrix of IMU readings (e.g., accel or gyro)
%   expected_vector    : 1x3 known true vector (e.g., [0 0 9.81])
%
% Outputs:
%   scale_matrix       : 3x3 identity (placeholder for scale/misalignment)
%   bias               : 1x3 estimated bias (avg - expected)
%   noise_var_matrix   : 3x3 diagonal matrix of variances (X, Y, Z)
%   apply_correction   : function handle to correct new IMU readings

    % Step 1: Estimate mean and bias
    bias = mean(raw_measurements - expected_vector, 1);
    % bias = avg_measurement - expected_vector;

    % Step 2: Compute per-axis noise variance
    zeroed = raw_measurements - bias - expected_vector;  % Remove gravity + bias
    noise_var = var(zeroed, 0, 1);                        % 1x3 vector
    noise_var_matrix = diag(noise_var);                  % 3x3 diagonal

    % Step 3: No scaling or misalignment assumed
    scale_matrix = eye(3);  % Placeholder

    % Step 4: Create correction function
    apply_correction = @(raw) (scale_matrix * (raw - bias)')';

end
%%

% function slam_w_cam()    
intrinsics = cameraIntrinsics([4.585664062500000e+02 4.584726562500000e+02],[3.170468750000000e+02 2.568281250000000e+02],[480 640]);
depthScaleFactor = 1/2.500000118743628e-04 %?
%depthScaleFactor = 1/0.00025

colorDataArray = {};
depthDataArray = {};

depth_struct = load("depth_and_color_data.mat");
depth_data = depth_struct.depthDataArray;
color_struct = load("color_data.mat");
color_data = color_struct.colorDataArray;
%%
accel_gyro_struct = load("accel_and_gyro_data.mat");
accel_data = accel_gyro_struct.accelDataArray;
gyro_data = accel_gyro_struct.gyroDataArray;


% % % IMU Calibration
% num_buffer= 1000;
% gyro_calibration = zeros(num_buffer, 3);
% accel_calibration = zeros(num_buffer, 3);
% for i=1:num_buffer
% 
%     accel_calibration(i, :) = accel_data{i};
%     gyro_calibration(i, :) = gyro_data{i};
% 
% 
%         % Show live running mean/variance
%         % accel_running_mean = mean(accel_buffer(1:min(i, num_buffer), :), 1);
% 
%     % pause(0.1)
% end
% % % Trim to valid samples
% 
% % Define expected vector (e.g., IMU Z axis up)
% accel_expected_vector = [0, 0, 9.81];
% gyro_expected_vector = [0, 0, 0];
% % 
% % Estimate bias and scale matrix
% [accel_scale_matrix, accel_bias, accel_var, accel_apply_correction] = calibrate_imu(accel_calibration, repmat(accel_expected_vector, size(accel_calibration,1), 1));
% [gyro_scale_matrix, gyro_bias, gyro_var, gyro_apply_correction] = calibrate_imu(gyro_calibration, repmat(gyro_expected_vector, size(gyro_calibration,1), 1));

%
%%

% IMU param
% sampleRate = 100; % Hz
% gyroBiasNoise = gyro_bias.*eye(3);
% accelBiasNoise = accel_bias.*eye(3);
% gyroNoise = gyro_var;
% accelNoise = accel_var;
% imuparams = factorIMUParameters(SampleRate=sampleRate, ...
%                                 GyroscopeBiasNoise=gyroBiasNoise, ...
%                                 AccelerometerBiasNoise=accelBiasNoise, ...
%                                 GyroscopeNoise=gyroNoise, ...
%                                 AccelerometerNoise=accelNoise, ...
%                                 ReferenceFrame="NED");
% depthToIMU = depth_profile.get_extrinsics_to(accel_profile);
% rotMat = reshape(depthToIMU.rotation, [3,3]);
% tMat = depthToIMU.translation;


imuStruct = load("imuParams.mat")
imuparams = imuStruct.imuparams

r_and_t_struct = load("camToIMUExtrinsics.mat");
rotMat = r_and_t_struct.rotMat;
tMat = r_and_t_struct.tMat;
cameraToIMU = rigidtform3d(rotMat,tMat);

disp("Calibration complete.");
% vslam = rgbdvslam(intrinsics,depthScaleFactor, imuparams, CameraToIMUTransform=cameraToIMU);
vslam = rgbdvslam(intrinsics, depthScaleFactor, imuparams, CameraToIMUTransform=cameraToIMU, TrackFeatureRange = [15, 100], LoopClosureThreshold=30, DepthRange = [.19, 9]);

%%
    for i = 1:length(depth_data)-1

        % Select depth frame
        color = cell2mat(color_data(i));
        depth = cell2mat(depth_data(i));
        a = accel_data{i};
        g = gyro_data{i};
        % 
        % if (depth.logical() && color.logical())
        %     % disp("Color and depth good")

            addFrame(vslam,color,depth, g, a);
        
            if hasNewKeyFrame(vslam)
                % Query 3-D map points and camera poses
                disp("keyframe_added")
                xyzPoints = mapPoints(vslam);
                [camPoses,viewIds] = poses(vslam);
        
                % Display 3-D map points and camera trajectory
                plot(vslam);
            end
            plot(vslam);
            % Get current status of system
            status = checkStatus(vslam);
            disp(status)
            % Stop adding frames when tracking is lost
            % if status == uint8(0)
            %     break
            % end
        % end
    end 
% end

%%

pc = pointCloud(xyzPoints);
player = pcplayer(pc.XLimits,pc.YLimits,pc.ZLimits);

view(player, pc)