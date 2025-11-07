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
depthScaleFactor = 1/2.500000118743628e-04; %?
cfg = realsense.config();
cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);    
cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);  
cfg.enable_stream(realsense.stream.gyro, realsense.format.motion_xyz32f, 100);
cfg.enable_stream(realsense.stream.accel, realsense.format.motion_xyz32f, 100);
pipe = realsense.pipeline();
colorizer = realsense.colorizer();
pointcloud = realsense.pointcloud();
cleanup = onCleanup(@() pipe.stop());  % Automatically called if function exits or errors
% Start streaming on an arbitrary camera with default settings
% profile = pipe.start(cfg);
profile = pipe.start(cfg);

% Enable motion correction
dev = profile.get_device();
sensors = dev.query_sensors();
for i = 1:length(sensors{1})
    sensor = sensors{1}{i};
    if sensor.is('depth_sensor')
        depth_scale = sensor.as('depth_sensor').get_depth_scale();
        visual_present_option_val = 5;     % Short-range option for depth sensor
        sensor.set_option(realsense.option.visual_preset, visual_present_option_val);   % Set to short-range
        fprintf('\rVisual present option: %s', sensor.get_option_value_description(realsense.option.visual_preset, visual_present_option_val));
    end
    if sensor.supports_option(realsense.option.enable_motion_correction)
        sensor.set_option(realsense.option.enable_motion_correction, 1)
    end
end

active_profiles = profile.get_streams();
% Find the accel and gyro profiles
for i = 1:numel(active_profiles)
    sp = active_profiles{i};
    if sp.stream_type() == realsense.stream.accel
        accel_profile = sp.as('motion_stream_profile');
        accel_intrinsics = accel_profile.get_motion_intrinsics();
    elseif sp.stream_type() == realsense.stream.gyro
        gyro_profile = sp.as('motion_stream_profile');
        gyro_intrinsics = gyro_profile.get_motion_intrinsics();
    elseif sp.stream_type() == realsense.stream.depth;
        depth_profile = sp.as('video_stream_profile');
        depth_intrinsics = depth_profile.get_intrinsics;
    elseif sp.stream_type() == realsense.stream.color;
        color_profile = sp.as('video_stream_profile');
        color_intrinsics = color_profile.get_intrinsics;
    else
        continue
    end
end
colorDataArray = {};
depthDataArray = {};

% IMU Calibration
num_buffer = 1000;
gyro_calibration = zeros(num_buffer, 3);
accel_calibration = zeros(num_buffer, 3);
for i=1:num_buffer
    fs = pipe.wait_for_frames();

    gyro_frame = fs.first(realsense.stream.gyro);
    gyro_motion_frame = gyro_frame.as('motion_frame');
    accel_frame = fs.first(realsense.stream.accel);
    accel_motion_frame = accel_frame.as('motion_frame');
    if (accel_frame.isvalid() && gyro_frame.isvalid())
        try
        a = accel_motion_frame.get_motion_data();
        g = gyro_motion_frame.get_motion_data();
        n = n + 1;
        % i = mod(i, num_buffer) + 1;  % cyclic index 1...num_buffer
        % accel_buffer(i, :) = [a(1), a(2), a(3)];
        % gyro_buffer(i, :) = [g(1), g(2), g(3)];
        accel_calibration(i, :) = a;
        gyro_calibration(i, :) = g;
        

        % Show live running mean/variance
        % accel_running_mean = mean(accel_buffer(1:min(i, num_buffer), :), 1);

        catch err
            warning("Skipping frame due to error: %s", err.message);
            continue;
        end
    end
    delete(fs);
    % pause(0.01)
    % Optional: plot real-time readings
    % plot3(accel_buffer(:,1), accel_buffer(:,2), accel_buffer(:,3), '.b'); drawnow;

    % pause(0.1)
end
% Trim to valid samples

% Define expected vector (e.g., IMU Z axis up)
accel_expected_vector = [0, 0, 9.81];
gyro_expected_vector = [0, 0, 0];
% 
% Estimate bias and scale matrix
[accel_scale_matrix, accel_bias, accel_var, accel_apply_correction] = calibrate_imu(accel_calibration, repmat(accel_expected_vector, size(accel_calibration,1), 1));
[gyro_scale_matrix, gyro_bias, gyro_var, gyro_apply_correction] = calibrate_imu(gyro_calibration, repmat(gyro_expected_vector, size(gyro_calibration,1), 1));

active_profiles = profile.get_streams();
for i = 1:numel(active_profiles)
    sp = active_profiles{i};
    if sp.stream_type() == realsense.stream.accel
        accel_profile = sp.as('motion_stream_profile');
        accel_intrinsics = accel_profile.get_motion_intrinsics();
    elseif sp.stream_type() == realsense.stream.gyro
        gyro_profile = sp.as('motion_stream_profile');
        gyro_intrinsics = gyro_profile.get_motion_intrinsics();
    elseif sp.stream_type() == realsense.stream.depth;
        depth_profile = sp.as('video_stream_profile');
        depth_intrinsics = depth_profile.get_intrinsics;
    elseif sp.stream_type() == realsense.stream.color;
        color_profile = sp.as('video_stream_profile');
        color_intrinsics = color_profile.get_intrinsics;
    else
        continue
    end
end

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
% cameraToIMU = rigidtform3d(rotMat,tMat);

disp("Calibration complete.");
% vslam = rgbdvslam(intrinsics,depthScaleFactor, imuparams, CameraToIMUTransform=cameraToIMU);
vslam = rgbdvslam(intrinsics,depthScaleFactor);

%%
    while(true)
        % Obtain frames from a streaming device
        fs = pipe.wait_for_frames();
        % Select depth frame
        color = fs.get_color_frame();
        depth = fs.get_depth_frame();
        gyro_frame = fs.first(realsense.stream.gyro);
        gyro_motion_frame = gyro_frame.as('motion_frame');
        accel_frame = fs.first(realsense.stream.accel);
        accel_motion_frame = accel_frame.as('motion_frame');
        a = accel_motion_frame.get_motion_data();
        g = gyro_motion_frame.get_motion_data();
        if (depth.logical() && color.logical())
            % disp("Color and depth good")
            depthData = depth.get_data();
            depthDataArray{end + 1} = depthData;
            % depthColor = colorizer.colorize(depth);
            depthImage = permute(reshape(depthData',[depth.get_width(),depth.get_height()]),[2 1]);
            % depthData = color.get_data();
            % depthImage = permute(reshape(depthData',[3,color.get_width(),color.get_height()]),[3 2 1]);

            % Get actual data and convert into a format imshow can use
            % (Color data arrives as [R, G, B, R, G, B, ...] vector)
            colorData = color.get_data();
            colorDataArray{end + 1} = colorData;
            colorImage = permute(reshape(colorData',[3,color.get_width(),color.get_height()]),[3 2 1]);
            
            addFrame(vslam,colorImage,depthImage, g, a);
        
            if hasNewKeyFrame(vslam)
                % Query 3-D map points and camera poses
                disp("keyframe_added")
                xyzPoints = mapPoints(vslam);
                [camPoses,viewIds] = poses(vslam);
        
                % Display 3-D map points and camera trajectory
                plot(vslam);
            end
            
            % Get current status of system
            status = checkStatus(vslam);
            disp(status)
            % Stop adding frames when tracking is lost
            % if status == uint8(0)
            %     break
            % end
        end
        delete(fs)
        pause(0.01);
    end 
    save('depth_and_color_data.mat', 'colorDataArray', 'depthDataArray');
    pipe.stop();
% end