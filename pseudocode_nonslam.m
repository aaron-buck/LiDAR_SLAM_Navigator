%% === Initialization ===

% 1. Calibrate IMU and LiDAR
imuCalib = calibrateIMU(getIMUData());        % Replace with actual IMU calibration
lidarCalib = calibrateLiDAR();                % Optional if needed for point alignment

% 2. Set horizontal width filter for forward obstacle zone
forwardWidth = 0.5; % meters (25 cm to left and right of center)
xMin = -forwardWidth / 2;
xMax = forwardWidth / 2;

% 3. Threshold parameters
d1 = 0.5;  % Very close (m)
d2 = 1.2;  % Close (m)
v1 = 0.6;  % Speed threshold (m/s)

% 4. Output control parameters
Amp1 = 5.0; % Very close buzz amplitude (V)
Amp2 = 3.0; % Close buzz amplitude (V)
b1 = 5;     % Fast approach buzz frequency (Hz)
b2 = 2;     % Slow approach buzz frequency (Hz)

% 5. Setup Arduino communication
arduinoPort = serialport("COM3", 9600); % Replace COM3 with your actual port

%% === Real-Time Monitoring Loop ===
while streamIsActive()
    %% Step 1: Process LiDAR Data
    if isNewLiDARAvailable()
        ptCloud = getLiDARPointCloud();  % Expecting Nx3 or pointCloud object
        lidarPts = ptCloud.Location;     % Nx3 matrix: [x, y, z]

        % Filter for points within forward horizontal corridor
        inForwardZone = lidarPts(:,1) >= xMin & lidarPts(:,1) <= xMax;
        forwardPoints = lidarPts(inForwardZone, :);

        % Determine distance to closest point
        if isempty(forwardPoints)
            category = "c3";  % obstacle-free
        else
            distances = vecnorm(forwardPoints, 2, 2);  % Euclidean distances
            minDist = min(distances);

            if minDist < d1
                category = "c1";  % very close
            elseif minDist < d2
                category = "c2";  % close
            else
                category = "c3";  % obstacle-free
            end
        end
    end

    %% Step 2: Set Buzzing Amplitude Based on Obstacle Distance
    switch category
        case "c1"
            amplitude = Amp1;
        case "c2"
            amplitude = Amp2;
        otherwise
            amplitude = 0;
    end

    %% Step 3: Process IMU Data for Relative Velocity
    if isNewIMUAvailable()
        imuData = getIMUData();               % raw IMU reading
        userVelocity = estimateForwardVelocity(imuData, imuCalib);  % function to compute velocity

        if category ~= "c3"  % Only adjust frequency if obstacle is ahead
            if userVelocity > v1
                frequency = b1;
            else
                frequency = b2;
            end
        else
            frequency = 0; % no buzzing if no obstacle
        end
    end

    %% Step 4: Send Buzz Command to Arduino
    % Format: "AMP:<amplitude>;FREQ:<frequency>\n"
    buzzCmd = sprintf("AMP:%.1f;FREQ:%d\n", amplitude, frequency);
    write(arduinoPort, buzzCmd, "string");

end
