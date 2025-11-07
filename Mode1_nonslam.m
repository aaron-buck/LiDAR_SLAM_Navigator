function [relevant_dists] = pool(lidar_points)
    lidar_points = reshape(lidar_points, [480, 640]);
    relevant_dists = [];
    for i = 1:5:475
        for j = 1:5:635
            this_block_med = mean(lidar_points(i:i+9, j:j+9), 'all');
            relevant_dists(end + 1) = this_block_med;
        end
    end
    return
end

function device = bluetoothWrapper(device_name)

    devlist = blelist();
    % disp(devlist);

    if find(devlist{:,"Name"} == device_name)
        device = ble(device_name);
        disp("BLE Device Found!");
    else
        error("Error: Specified Bluetooth Device Not Found")
    end

end

function [corrected_gain_ratio, mean_offset, apply_correction] = calibrate_imu(raw_measurement, expected_vector, threshold)
% calibrate_imu - Calibrates IMU by estimating bias and scale/misalignment matrix.
%
% Inputs:
%   raw_measurements : Nx3 matrix of IMU readings (e.g., accel or gyro)
%   expected_vectors : Nx3 matrix of known true vectors (e.g., gravity = [0 0 9.81])
%
% Outputs:
%   scale_matrix     : 3x3 scale + misalignment matrix
%   bias             : 1x3 vector of biases
%   apply_correction : function handle to apply correction: corrected = f(raw)

    % Estimate bias as difference from expected
    mean_offset = mean(raw_measurement - expected_vector, 1);
    zeroed = raw_measurement - mean_offset - expected_vector;

    % Estimate skewness in data fluctuating by correcting the gain
    % coefficient
    mean_pos = mean(zeroed(zeroed > 0));
    mean_neg = mean(abs(zeroed(zeroed < 0)));
    gain_ratio = mean_pos / mean_neg;
    corrected_gain_ratio = 1 / gain_ratio;

    % Create function handle to apply correction
    apply_correction = @(raw) corrected_or_zero(raw);

    function result = corrected_or_zero(raw)
        corrected = raw - corrected_gain_ratio * mean_offset - expected_vector;

        if abs(corrected) < threshold
            result = 0;  % Return zero vector
        else
            result = corrected;
        end
    end
end
%% === Initialization ===

cfg = realsense.config();
cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);    
cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);  
cfg.enable_stream(realsense.stream.gyro);
cfg.enable_stream(realsense.stream.accel);

pipe = realsense.pipeline();
profile = pipe.start(cfg);
colorizer = realsense.colorizer();

% Get depth scale and enable automatic motion correction
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
        sensor.set_option(realsense.option.enable_motion_correction, 0)
    end
end

% Find the all data's intrinsics
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

% Manual IMU calibration
n = 200;
gyro_calibration = zeros(n, 1);
accel_calibration = zeros(n, 1);
for i = 1:n
    fs = pipe.wait_for_frames();
    gyro_frame = fs.first(realsense.stream.gyro);
    gyro_motion_frame = gyro_frame.as('motion_frame');
    accel_frame = fs.first(realsense.stream.accel);
    accel_motion_frame = accel_frame.as('motion_frame');
    if (accel_frame.isvalid() && gyro_frame.isvalid())
        try
        a = norm(accel_motion_frame.get_motion_data());
        g = norm(gyro_motion_frame.get_motion_data());
        accel_calibration(i,:) = a;
        gyro_calibration(i,:) =  g;        
        catch err
            warning("Skipping frame due to error: %s", err.message);
            continue;
        end
    end
    delete(fs);
    pause(0.01)
end
% Define expected vector (e.g., IMU Z axis up)
accel_expected_vector = [9.81];
gyro_expected_vector = [0];

% Estimate bias and scale matrix
[accel_scale_matrix, accel_bias, accel_apply_correction] = calibrate_imu(accel_calibration, repmat(accel_expected_vector, size(accel_calibration,1), 1), 0.05);
[gyro_scale_matrix, gyro_bias, gyro_apply_correction] = calibrate_imu(gyro_calibration, repmat(gyro_expected_vector, size(gyro_calibration,1), 1), 0.05);
disp("Calibration complete.");

% Initialize camera intrinsics
fx = depth_intrinsics.fx;
fy = depth_intrinsics.fy;
ppx = depth_intrinsics.ppx;  % principal point x
ppy = depth_intrinsics.ppy;  % principal point y
[depth_width, depth_height] = deal(depth_intrinsics.width, depth_intrinsics.height);
depth_intrinsics_obj = cameraIntrinsics([fx fy], [ppx ppy], [double(depth_width), double(depth_height)]);

% Initialize velocity
forward_vel = 0;

% Initialize number of pixels to be pooled to calculate minimum distance
min_pix_ct = 1000;

% 1. Set horizontal width filter for forward obstacle zone 
% (TODO: check if this still holds)
forwardWidth = 0.4; % meters (25 cm to left and right of center)
xMin = -forwardWidth / 2;
xMax = forwardWidth / 2;

yMax = -0.5;

recent_distances = zeros(1,5);

min_z_threshold = 0.1;

% 3. Threshold parameters (TODO: tune parameters)
d1 = 1.5;  % Very close (m)
d2 = 1.2;  % Close (m)
d3 = 0.8;
v1 = 0.6;  % Speed threshold (m/s)

% 4. Arduino Initialization
clear arduino;
arduino = bluetoothWrapper("Haptic");
distanceChar = characteristic(arduino, "19B10010-E8F2-537E-4F6C-D104768A1214", "19B10001-E8F2-537E-4F6C-D104768A1214");
speedChar = characteristic(arduino, "19B10010-E8F2-537E-4F6C-D104768A1214", "19B10012-E8F2-537E-4F6C-D104768A1214");

% 5. Setup Arduino communication (TODO: correct syntax)
figure;
% Subplot 1
subplot(1,2,1);
h1 = image(zeros(480, 640, 3, 'uint8'));
title('Filtered Colorized');
axis image off;

% Subplot 2
subplot(1,2,2);
h2 = bar(0);
title('Buzzing Amplitude');
ylim([0 4]); 
% timestep = 0;
%% === Real-Time Monitoring Loop ===
while true
    fs = pipe.wait_for_frames();
    
    depth = fs.get_depth_frame();
    
    % Get motion data
    gyro_frame = fs.first(realsense.stream.gyro);
    accel_frame = fs.first(realsense.stream.accel);
    gyro_motion_frame = gyro_frame.as('motion_frame');
    accel_motion_frame = accel_frame.as('motion_frame');
    g = gyro_motion_frame.get_motion_data();
    a = norm(accel_motion_frame.get_motion_data());
    accel = accel_apply_correction(a);
    forward_vel = forward_vel + accel * 0.05;

    % Debug purposes
    % fprintf('\rresidual accel after offset correction: %.2f', accel)
    % fprintf('\rresidual accel before offset correction: %.2f', a)
    % fprintf('\rForward vel: %.2f', forward_vel);

    %% Step 1: Process LiDAR Data
    depthData = depth.get_data();
    depthImage = permute(reshape(depthData',[depth.get_width(),depth.get_height()]),[2 1]);
    ptCloud = pcfromdepth(depthImage, 1/depth_scale, depth_intrinsics_obj);  % Expecting Nx3 or pointCloud object
    lidarPts = ptCloud.Location;     % Nx3 matrix: [x, y, z]

    % Filter for points within forward horizontal corridor
    inForwardZone_x = lidarPts(:,:,1) >= xMin & lidarPts(:,:,1) <= xMax & lidarPts(:,:,1) ~= 0;
    inForwardZone_y = lidarPts(:,:,2) >= yMax & lidarPts(:,:,2) ~= 0;
    % inForwardZone_z = lidarPts(:,:,3) <= 1.2;
    inForwardZone = (inForwardZone_x & inForwardZone_y);
    forwardPoints = reshape(inForwardZone, [], 1);
    flattened_lidarPts = reshape(lidarPts, [], 3);

    flattened_lidarPts(~forwardPoints, 3) = inf; 
    relevant_dist = pool(flattened_lidarPts(:,3));
    relevant_dist = relevant_dist(relevant_dist >= min_z_threshold);
    if isempty(relevant_dist)
        relevant_dist = [inf];
    end
    % recent_distances = circshift(recent_distances, 1);
    min_dist = mink(relevant_dist, 1);
    % min_dist = mean(recent_distances);
    fprintf('\rMin dist: %.2f \r', min_dist);
    % smth = reshape(flattened_lidarPts, [480,640,3]);
    % smth_z = smth(:,:,3);
    % [r_row, r_col] = find(smth_z ~= 0);

    % Comparison purposes between full image versus when restricted with
    % x-distance
    color = colorizer.colorize(depth);
    % Get actual data and convert into a format imshow can use
    % (Color data arrives as [R, G, B, R, G, B, ...] vector)
    depth_colorized_data = color.get_data();
    img = permute(reshape(depth_colorized_data',[3,color.get_width(),color.get_height()]),[3 2 1]);
    flattened_depth_colorized = reshape(img, [], 3);
    flattened_depth_colorized(~forwardPoints,:) = 0;
    filtered_img = reshape(flattened_depth_colorized, [480, 640, 3]);
    

    % Display image
    set(h1, 'CData', filtered_img);
    

    %% write to arduino
    velocity = 0.1;

    % Determine distance to closest point
    if isempty(forwardPoints)
        category = "c4";  % obstacle-free
    else
        % distances = vecnorm(forwardPoints, 2, 2);  % Euclidean distances
        % min_dist = min(distances);

        if min_dist < d3
            category = "c3";  % kinda far
            disp('c3')
        elseif min_dist < d2
            category = "c2";  % close
            disp('c2')
        elseif min_dist < d1 
            category = "c1";  % very closeclose
            disp('c1')
        else
            category = "c4";  % obstacle-free
            disp('c4')
        end
    end

    %% Step 2: Set Buzzing Amplitude Based on Obstacle Distance
    switch category
        case "c1"
            amplitude = 1;
        case "c2"
            amplitude = 2;
        case "c3"
            amplitude = 3;
        otherwise
            amplitude = 0;
    end
    
    % x(end+1) = timestep;
    % y(end+1) = amplitude;
    set(h2, 'YData', amplitude); %, 'YData', y);
    drawnow;

    %% Step 3: Process IMU Data for Relative Velocity
    % if isNewIMUAvailable()

        if category ~= "c3"  % Only adjust frequency if obstacle is ahead
            if velocity > v1
                frequency = 2; %% 5 Hz
            else
                frequency = 1; %% 2 Hz
            end
        else
            frequency = 0; % no buzzing if no obstacle
        end
    % end

    %% Step 4: Send Buzz Command to Arduino
    % Format: "AMP:<amplitude>;FREQ:<frequency>\n"

    distance = amplitude;
    write(distanceChar, distance);
    speed = frequency;
    write(speedChar, speed);
    % pause(0.05)

end
