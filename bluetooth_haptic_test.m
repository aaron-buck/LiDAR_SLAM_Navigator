
% BLE setup

clear arduino;
arduino = bluetoothWrapper("Haptic");
distanceChar = characteristic(arduino, "19B10010-E8F2-537E-4F6C-D104768A1214", "19B10001-E8F2-537E-4F6C-D104768A1214");
speedChar = characteristic(arduino, "19B10010-E8F2-537E-4F6C-D104768A1214", "19B10012-E8F2-537E-4F6C-D104768A1214");

%% write to arduino

% distance = ??;
write(distanceChar, distance);
% speed = ??;
write(speedChar, speed);

% pause(0.5);


%%

while 1
    val = randi(3);
    write(distanceChar, val);

    pause(0.5);
end
