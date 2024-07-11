client = tcpclient("IP", 30000);
disp("Connected");
configureTerminator(client, "LF", "CR/LF");

running = true;
maxRange = 19.2; % meters
resolution = 10; % cells per meter

slamObj = lidarSLAM(resolution, maxRange, 1000);
slamObj.LoopClosureThreshold = 400;
slamObj.LoopClosureSearchRadius = 10;

counter = 9; % Initialize counter variable
scans = 0;


refAngles = zeros(1, 43);
refDistances = zeros(1, 43);
refscan = lidarScan(refDistances, refAngles);
while true
    while client.NumBytesAvailable > 0
        % Read a line of JSON data from the client
        msg = readline(client);
        counter = counter + 1; % Increment counter
        disp(scans);
        if mod(counter, 15) == 0
            % Process the message every 10th iteration
            % Decode the JSON data into a MATLAB array
            mapdata = jsondecode(msg);
            disp(class(mapdata)); % Display the class of the decoded data
            disp("Length of mapdata:");
            disp(length(mapdata));

            % Check if the length of mapdata is divisible by 4
            if mod(length(mapdata), 4) ~= 0
                disp("Received data length is not divisible by 4.");
                continue; % Skip processing this data and move to the next iteration
            end

            % Preallocate arrays for angles and distances
            angles = zeros(1, length(mapdata) / 4);
            distances = zeros(1, length(mapdata) / 4);

            % Extract angles and distances from mapdata
            for i = 1:length(mapdata)/4
                angle_high_byte = mapdata((i - 1) * 4 + 1);
                angle_low_byte = mapdata((i - 1) * 4 + 2);
                distance_high_byte = mapdata((i - 1) * 4 + 3);
                distance_low_byte = mapdata((i - 1) * 4 + 4);

                angle = bitor(uint16(angle_low_byte), bitshift(uint16(angle_high_byte), 8));
                distance = bitor(uint16(distance_low_byte), bitshift(uint16(distance_high_byte), 8));

                % Convert angle from degrees to radians and normalize to [-pi, pi]
                angle_rad = deg2rad(double(angle) / 100);
                angle_rad = mod(angle_rad + pi, 2 * pi) - pi;

                % Convert distance to meters
                distance_m = double(distance) / 10000; % Convert from mm to m 

                angles(i) = angle_rad;
                distances(i) = distance_m;                   
            end

            % Filter out invalid distance values and apply additional filtering
            valid_indices = (distances > 0) & (distances < maxRange);
            angles = angles(valid_indices);
            distances = distances(valid_indices);

            % Verify the filtered angles and distances
            disp("Filtered distances in meters:");
            disp(distances);
            disp("Length of the filtered distances array:"); 
            disp(length(distances));
            disp("Filtered angles in radians:"); 
            disp(angles);
            disp("Length of the filtered angles array:");
            disp(length(angles));

            % Apply the custom median filter to reduce noise
            % ???????????????????????????????
            distances = medianFilter(distances, 3);

            % Create a lidarScan object if valid data is available
            if ~isempty(distances) && ~isempty(angles)
                lidarScanObject = lidarScan(distances, angles);
                %plot(lidarScanObject); % Plot individual scan to verify its correctness
                %drawnow;

                % Add the scan to the lidarSLAM object and handle potential errors
                try
                    pose = matchScans(lidarScanObject, refscan);
                    %pose = [0 0 0];
                    lidarScanObject = transformScan(lidarScanObject, pose);                    % addScan(slamObj, lidarScanObject ,pose);        % pose = addScan(slamObj, lidarScanObject);
                    addedScan = addScan(slamObj, lidarScanObject);
                    refscan = lidarScanObject;
                    % Check if the pose is valid to confirm the scan addition
                    if ~isempty(addedScan)
                        disp("Scan added successfully.");
                    else
                        disp("Failed to add scan.");
                    end

                    % Optionally, display the SLAM map and trajectory
                    try
                        figure(1); % Ensure the figure is specified
                        show(slamObj);
                    catch ME
                        disp("Error displaying SLAM object:");
                        disp(ME.message);
                    end

                catch ME
                    disp("Error adding scan to SLAM object:");
                    disp(ME.message);
                    % Additional handling, like logging or adjusting parameters, can be done here
                end

                % Reset the counter after processing every 10th message
                counter = 0;  
                scans = scans + 1;
            else
                disp("No valid data to create a lidarScan object.");
            end
        end
    end
end