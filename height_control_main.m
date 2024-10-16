close all;  
clear all;
clc;

%% Documentation variables
ExperimentNum = 0;
DroneNum = 0;
ExperimentType = "";

load("AFOSR_Results\1-C_Metadata.mat",'ExperimentNum','DroneNum','ExperimentType');

ExperimentNum = ExperimentNum + 1;

expStr = int2str(ExperimentNum);
droneStr = int2str(DroneNum);

filename = sprintf("AFOSR_Results/%s-%s_Metadata", droneStr, ExperimentType);

save(filename, 'ExperimentNum', 'DroneNum', 'ExperimentType');

timeNow = "mm-dd-yy_HH-MM";

filenameDate = datestr(now, timeNow);

filename = sprintf("AFOSR_Results/%s-%s-%s_%s",droneStr,ExperimentType,expStr, filenameDate);


% 2-C-3_mm-dd-yy_HH-MM
% DroneID-TypeofExp-ExpNum_Date_Time

if(ExperimentType == "C")
    TRAJECTORY = 0; % 0 = circular, 1 = origin reference
else
    TRAJECTORY = 1; % 0 = circular, 1 = origin reference
end


%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP); 

%% Connect to the Drone via Radio
%Make sure to get the the Drone's MAC address before running this code -
%b = ble("C0286E324A33"); % ST DRONE FRAME 1
b = ble("C0283C361730"); % FOAM CORE FRAME 1

char = b.Characteristics; % Get the characteristics of the Drone

%device = serialport("COM4",9600)
%flush(device)
%startByte = 245; 
%endByte = 2;

%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.killswitch

Drone_ID = 1;
%Drone_ID = 2;

%% Store the reference to the Charactersitic responsible for Writing Joystick data:
% https://www.st.com/resource/en/user_manual/dm00550659-getting-started-with-the-bluest-protocol-and-sdk-stmicroelectronics.pdf
        % To send Joydata you need to send an array of 7 elements:
        % First element:- No idea what this does.
        % Second element:- rudder value (gRUD) 128 is like sending 0 (YAW)
        % Third element:- thrust value (gTHR) (THRUST)
        % Fourth element:- AIL value (gAIL) 128 is like sending 0  (ROLL)
        % Fifth element:- ELE value (gELE) 128 is like sending 0  (PITCH)
        % Sixth element:- SEEKBAR value: mainly from android app just send 0.
        % Seventh element:- ARMING and CALIB: 0x04, 0x05, 0x01(For calib)
joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B"); % Write w/out response
% joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU
%joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU

%% Next calibrate/arm the drone
% Reference slides 25,26 from - https://www.st.com/content/ccc/resource/sales_and_marketing/presentation/product_presentation/group0/bd/cc/11/15/14/d4/4a/85/STEVAL-DRONE01_GETTING_STARTED_GUIDE/files/steval-drone01_getting_started_guide.pdf/jcr:content/translations/en.steval-drone01_getting_started_guide.pdf

% 1) Place drone down flat and press reset button to calibrate it
% 2) Arm the drone: 
write(joy_c, [0, 0, 0, 0, 0, 0, 2], 'uint8', "WithoutResponse") % is code running on drone different? Should send a 4 to arm?
java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
write(joy_c, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");

%% Set up data collection vectors
ITERATIONS = 2500 % Main loop time period
WARMUP = 100; % Filter warmup time period

% Data collection vectors
Drone_data = zeros(ITERATIONS + 2, 7);  % drone position data
sent_data = zeros(ITERATIONS + 1, 3); % data sent to drone
cont_actual_data = zeros(ITERATIONS + 1, 9);
errors = [];

% X,Y,Z position data
p_x = zeros(ITERATIONS + 1, 1);
p_y = zeros(ITERATIONS + 1, 1);
p_z = zeros(ITERATIONS + 1, 1);

% Filtered velocity data
f_v_x = zeros(ITERATIONS + 1, 1);
f_v_y = zeros(ITERATIONS + 1, 1);
f_v_z = zeros(ITERATIONS + 1, 1);

%% Frequencies
OUT_FREQ = 60; % 60Hz write only
CUT_OFF_FREQ_VEL = 10;
CUT_OFF_FREQ_POS = 10;

%% Mass of the drone
m = 69.89/1000;
MAX_ANGLE = 27.73;

%% Inititalize the PID controllers
X_pid = Xpid_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);
Y_pid = Ypid_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);
Z_pid = Zpid_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);

%% Get the Drone data
[DronePos] = GetDronePosition(theClient, Drone_ID);
Drone_data(1, :) = DronePos;

%% SET POINT TO TRACK
x_ref = 0.0;
y_ref = 0.0;
z_ref_final = 0.6; % 0.5 meter (500mm) % 0.005
comm_yaw_d = 128; % integer representation of 128 is 0 degrees. Min max is 30 degrees


%% Initialize lowpass filter
[DronePos] = GetDronePosition(theClient, Drone_ID);
lpfData_x = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(2));
lpfData_y = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(3));
lpfData_z = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(4));

lpfData_vx = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vy = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vz = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

prev_x =  DronePos(2);
prev_y =  DronePos(3);
prev_z =  DronePos(4);

%% Warm up Filter
for i = 1:WARMUP
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    %Drone_data = [Drone_data; DronePos];
    
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, x_f - prev_x);
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, y_f - prev_y);
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, z_f - prev_z);
    prev_z = z_f;
    
end

%% Run Main Loop

disp("Starting")
java.lang.Thread.sleep(5*1000); % Wait 5 seconds

T_trim = 100;

k = 1;
dT = 1/60; % 55Hz (writing only) - look into if dT might be faster 
wTimes          = [];
rTimes          = [];
loopTimes       = [];
sleepTimes      = [];
getPosTimes     = [];

[prevDronePos] = GetDronePosition(theClient, Drone_ID);
data = zeros(ITERATIONS+1,20); % For reading IMU
timestamps = datetime(zeros(10,1), 0, 0); %a 10x1 array of datetime
Drone_pos_data = [];
Drone_rate_data = [];

z_ref_final = 0.7;
xRefs = [];
yRefs = [];
zRefs = [];


flag1 = 0;
flag2 = 0;
flag3 = 0;

while(k <= ITERATIONS)
%     if(k>1500)
%         pause(0.1)
%     end
    %read(joy_c_imu);

    startT = tic;   
    
    % Get new drone position and store
    startTPos = tic;
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    getPosTimes(k) = toc(startTPos);    
    Drone_data(k+1, :) = DronePos;
    
%     Drone_pos_data(k,:) = DronePos(5:7);
%     Drone_rate_data(k,:) = (DronePos(5:7) - prevDronePos(5:7))/dT;
%     prevDronePos = DronePos;
    

    % Apply low pass filter to position/velocity measurements
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    p_x(k) = x_f;
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, (x_f - prev_x)/dT);
    f_v_x(k) = vx_f;
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    p_y(k) = y_f;
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, (y_f - prev_y)/dT);
    f_v_y(k) = vy_f;
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    p_z(k) = z_f;
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, (z_f - prev_z)/dT);
    f_v_z(k) = vz_f;
    prev_z = z_f;
    
    
    
   
    % if K<750 do circ traj, else do 0 origin ref
    if(TRAJECTORY == 0)
        % Circular trajectory
        x_ref = 0.5*cos(0.01*k);
    else
        % Position hover trajectory
        x_ref = x_f - sign(x_f)*0.25;
        if(x_f < 0.25 && x_f > -0.25)
            flag1 = 1;
            x_ref = 0;
        end
        if(flag1==1)
            x_ref = 0;
        end
    end
    
    

    % Store the refs
    xRefs(k) = x_ref;
    % Call the X Controller - Desired Roll
    [ddot_x_d, pid_output_x, X_pid] = Xcontroller(X_pid, x_ref, x_f, vx_f, dT, 1/CUT_OFF_FREQ_VEL,k);
    
    
    if(TRAJECTORY == 0)
        % Circular trajectory
        y_ref = 0.5*sin(0.01*k);
    else
        % Position hover trajectory
        y_ref = y_f - sign(y_f)*0.25;
        if(y_f < 0.25 && y_f > -0.25)
            flag2 = 1;
            y_ref = 0;
        end
        if(flag2==1)
            y_ref = 0;
        end
    end
    
    

    % Store the refs
    yRefs(k) = y_ref;
    % Call the Y Controller - Desired Pitch
    [ddot_y_d, pid_output_y, Y_pid] = Ycontroller(Y_pid, y_ref, y_f, vy_f, dT, 1/CUT_OFF_FREQ_VEL,k);
    
    
%     z_ref = z_f + 0.15;
%     if(z_f > z_ref_final - 0.2)
%         flag3 = 1;
%         z_ref = z_ref_final;
%     end
%     if(flag3==1)
%         z_ref = z_ref_final;
%     end
    
    % Store the refs
    zRefs(k) = z_ref_final;
    % Call the Z Controller - Desired Thrust
    [gTHR, pid_output_z, Z_pid] = Zcontroller(Z_pid, z_ref_final, z_f, vz_f, dT, 1/CUT_OFF_FREQ_VEL,k);
    
    % Apply trim input thrust
    comm_thr_d = gTHR + T_trim;
    
    % Calculate desired roll,pitch angles - From Harsh Report
    psi = DronePos(7); % yaw
%     psi = 0;
    phi_d = -m/single(comm_thr_d) * (ddot_x_d*cos(psi) + ddot_y_d*sin(psi))* 180/pi; % MIGHT NEED TO REPLACE comm_thr_d with actual thrust sent to actuators on drone
    theta_d = m/single(comm_thr_d) * (-ddot_y_d*cos(psi) + ddot_x_d*sin(psi)) * 180/pi;
%     theta_d = 0.5; % Dont command y-pitch yet (0.5 offset)
%     phi_d = 0.5; % Dont command x-roll yet
    
    % Cap angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);
    
    % Convert the angles to 0 - 255
    slope_m = 255.0/(MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m *(phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m *(theta_d + MAX_ANGLE));
    
    %Send the command to the Drone
    wTime = tic;
    write(joy_c, [0, comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d, 0, 5], 'uint8', "WithoutResponse") % ~18ms
    %write(device,[comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d],"uint8")
    wTimes(k) = toc(wTime);
    
    
    
    
    
    % READ AHRS DATA FROM DRONE
     %[data(k,:), timestamps(k)] = 
     %sensorValues(k,:) = parfeval(backgroundPool,@readIMU,1,joy_c_imu);

     %joy_c_imu.DataAvailableFcn = @displayCharacteristicData;
     %[thx,thy,thz] = parse_ble_euler(data(k,3:8),10);
     %euler(k,:) = [thx,thy,thz];
     %[thx_rate,thy_rate,thz_rate] = parse_ble_euler(data(k,9:14),100);
     %euler_rates(k,:) = [thx_rate,thy_rate,thz_rate];
    
    
    
    
    % Collect the data being sent
    errors(k) = Y_pid.y_curr_error;
    sent_data(k, :) = [comm_thr_d, comm_phi_d, comm_theta_d];
    cont_actual_data(k, :) = [pid_output_x, pid_output_y, pid_output_z];
    %java.lang.Thread.sleep(10); % 10ms delay
    
    loopTimes(k) = toc(startT);
%     dT = loopTimes(k);
    
    k = k+1;
end






% Land at the origin
x_ref = 0;
y_ref = 0;

disp("Landing")
% Landing phase
while(z_f > 0.1)
    startT = tic;
    

    z_ref = z_f - 0.10;

    % Get new drone position and store
    startTPos = tic;
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    getPosTimes(k) = toc(startTPos);    
    Drone_data(k+1, :) = DronePos;

    % Apply low pass filter to position/velocity measurements
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    p_x(k) = x_f;
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, (x_f - prev_x)/dT);
    f_v_x(k) = vx_f;
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    p_y(k) = y_f;
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, (y_f - prev_y)/dT);
    f_v_y(k) = vy_f;
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    p_z(k) = z_f;
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, (z_f - prev_z)/dT);
    f_v_z(k) = vz_f;
    prev_z = z_f;
    
    
    % Call the X Controller - Desired Roll
    [ddot_x_d, pid_output_x, X_pid] = Xcontroller(X_pid, x_ref, x_f, vx_f, dT, 1/CUT_OFF_FREQ_VEL,k);
    
    % Call the Y Controller - Desired Pitch
    [ddot_y_d, pid_output_y, Y_pid] = Ycontroller(Y_pid, y_ref, y_f, vy_f, dT, 1/CUT_OFF_FREQ_VEL,k);
    
    % Call the Z Controller - Desired Thrust
    [gTHR, pid_output_z, Z_pid] = Zcontroller(Z_pid, z_ref, z_f, vz_f, dT, 1/CUT_OFF_FREQ_VEL,k);
    
    % Apply trim input thrust
    comm_thr_d = gTHR + T_trim;
    
    % Calculate desired roll,pitch angles - From Harsh Report
    psi = DronePos(7); % yaw
%     psi = 0;
    phi_d = -m/single(comm_thr_d) * (ddot_x_d*cos(psi) + ddot_y_d*sin(psi))* 180/pi; % MIGHT NEED TO REPLACE comm_thr_d with actual thrust sent to actuators on drone
    theta_d = m/single(comm_thr_d) * (-ddot_y_d*cos(psi) + ddot_x_d*sin(psi)) * 180/pi;
%     theta_d = 0.5; % Dont command y-pitch yet - ADD IN OFFSET
%     phi_d = 0.5; % Dont command x-roll yet
    
    % Cap angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);
    
    % Convert the angles to 0 - 255
    slope_m = 255.0/(MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m *(phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m *(theta_d + MAX_ANGLE));
    
    %Send the command to the Drone
    wTime = tic;
     write(joy_c, [0, comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d, 0, 5], 'uint8', "WithoutResponse") % ~18ms
   % write(device,[comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d],"uint8")
    wTimes(k) = toc(wTime);
    
    % Collect the data being sent
    errors(k) = Y_pid.y_curr_error;
    sent_data(k, :) = [comm_thr_d, comm_phi_d, comm_theta_d];
    cont_actual_data(k, :) = [pid_output_x, pid_output_y, pid_output_z];
    
    %java.lang.Thread.sleep(10); % 10ms delay

    loopTimes(k) = toc(startT);
%     dT = loopTimes(k);
 
    k = k+1;
    
end





% Shut off drone
disp("Shutting Down")
write(joy_c, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");
%write(device,[startByte, 128, 0, 128, 128, endByte],"uint8")
% ik = 0;
% while ik < 10
%     %% QUIT
%     write(joy_c, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");
%     java.lang.Thread.sleep(100);
%     ik = ik+1;
% end
% Finally, Close the Motive Client
theClient.Uninitialize();

disp('Done')

% Saving Workspace to file
save(filename)


%% Display Results
close all

%disp(sensorValues)

figure()
plot(Drone_data(:,5)*180/pi)
title("Pitch");

figure()
plot(Drone_data(:,6)*180/pi)
title("Roll");

figure()
hold on;
plot(p_z)
plot(z_ref_final*ones(size(p_z,1)));
title("Height");
legend("Z", "Z_ref")
axis([0 ITERATIONS 0 2])
grid on;

figure()
hold on;
plot(p_x)
plot(x_ref*ones(size(p_x,1)));
title("X");
legend("X", "X_ref")
axis([0 ITERATIONS -2 2])

figure()
hold on;
plot(p_y)
plot(y_ref*ones(size(p_y,1)));
title("Y");
legend("Y", "Y_ref")
axis([0 ITERATIONS -2 2])

figure()
plot(p_x,p_y)
axis([-2 2 -2 2])

% PID Output X
figure()
plot(cont_actual_data(:,1:3))
title("PID X Output");
legend("p","i","d")

% PID Output Y
figure()
plot(cont_actual_data(:,4:6))
title("PID Y Output");
legend("p","i","d")

% PID Output Z
figure()
plot(cont_actual_data(:,7:9))
title("PID Z Output");
legend("p","i","d")
% 
% 
figure()
plot(sent_data)
title("Commands Sent");
legend("comm_thr", "comm_phi", "comm_theta")
% 
% figure()
% plot(errors)

figure()
plot(loopTimes)



% Find timeshift (and corresponding time delay) through xcorr (time delay should be 10-15ms (ie. = read() time?))
[c,lags] = xcorr(euler(:,1),Drone_pos_data(:,1)*(180/pi));
timeShift = find(c==max(c)) - ITERATIONS; % This number corresponds to how many ms?
timeShift = 0;
RMSE_pitch_angle = sqrt(mean((euler(1+timeShift:end,1) - Drone_pos_data(1:end-timeShift,1)*(180/pi)).^2))  % Root Mean Squared Error
% Plot with timeshift
figure();
plot(euler(1+timeShift:end,1));
hold on;
plot(Drone_pos_data(1:end-timeShift,1)*(180/pi))
legend("drone thx-pitch","motive thx-pitch");


% Find timeshift
[c,lags] = xcorr(euler(:,2),Drone_pos_data(:,2)*(180/pi));
timeShift = find(c==max(c)) - ITERATIONS; % This number corresponds to how many ms?
timeShift = 0;
RMSE_roll_angle = sqrt(mean((euler(1+timeShift:end,2) - Drone_pos_data(1:end-timeShift,2)*(180/pi)).^2))  % Root Mean Squared Error
% Plot with timeshift
figure();   
plot(euler(1+timeShift:end,2));
hold on;
plot(Drone_pos_data(1:end-timeShift,2)*180/pi)
legend("drone thy-roll","motive thy-roll");


pitchRatesFil_motive = lowpass(euler_rates(:,1)*180/pi,0.1);
pitchRatesFil_drone = lowpass(Drone_rate_data(:,1)*180/pi,0.1);
figure();
plot(pitchRatesFil_motive);
hold on;
plot(pitchRatesFil_drone)
legend("drone thx-pitch rate","motive thx-pitch rate");

rollRatesFil_motive = lowpass(euler_rates(:,2)*180/pi,0.1);
rollRatesFil_drone = lowpass(Drone_rate_data(:,2)*180/pi,0.1);
figure();
plot(rollRatesFil_motive);
hold on;
plot(rollRatesFil_drone)
legend("drone thy-roll rate","motive thy-roll rate");




