function device = bluetoothWrapper(device_name)

    devlist = blelist();
    disp(devlist);

    if find(devlist{:,"Name"} == device_name)
        device = ble(device_name);
    else
        error("Error: Specified Bluetooth Device Not Found")
    end

end