% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code
clear all;
clc;

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
%clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,1);

% connection status
connected = false;

% robot parameters
d = 0.1950; % wheel radius
r = d/2; % wheel radius
T = 0.3310;% wheel track

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % start simulation
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    % IMU signals
    [gyro_err,gyro_signal]=vrep.simxReadStringStream(clientID,'gyro_data',vrep.simx_opmode_streaming);
    [accel_err,accel_signal]=vrep.simxReadStringStream(clientID,'accel_data',vrep.simx_opmode_streaming);
    
end

% call figure
%--------------------------------------------------------------------
environment(); %uncomment this line to plot in the map
%--------------------------------------------------------------------
line = line(nan, nan, 'color', 'red');


%% initial values

Ax(1) = 0;
Vx(1) = 0;
Vy(1) = 0;
Dx(1) = 0;
Dy(1) = 0;
t(1)  = 0;
PosX(1)  = 5;
PosY(1)  = 2;
Ar(1) = 0; 
Vr(1) = 0;
Pr(1) = 0;
theta(1) = pi;
i = 1;
cmdTime = 0;
dth = 0;
dthG = 0;
samplingTime = 0.05;
cutoff_freqL = 0;
cutoff_freqH = 0;

smooth_factL = 0
smooth_factH = 0



 if (connected)   
     %% Program start
     
      while (cmdTime < 420000) % CHANGE THIS LINE TO 'while loop'
      %while (1) % CHANGE THIS LINE TO 'while loop'

        
        % Retrieves the simulation time of the last fetched command 
        [cmdTime] = vrep.simxGetLastCmdTime(clientID); % TIME IF YOU NEED IT
        
        % read gyroscope and accelerometer data
        [gyro_err,gyro_signal]=vrep.simxReadStringStream(clientID,'gyro_data',vrep.simx_opmode_buffer);
        [accel_err,accel_signal]=vrep.simxReadStringStream(clientID,'accel_data',vrep.simx_opmode_buffer);
        
        % Gyroscope
        if gyro_err == vrep.simx_return_ok
            [gyro_buffer]= vrep.simxUnpackFloats(gyro_signal);
                
            if isempty(gyro_buffer)
                continue
            end
            
        else
            continue;
        end
        
        % Accelerometer
        if accel_err == vrep.simx_return_ok
            [accel_buffer]= vrep.simxUnpackFloats(accel_signal);
            
            if isempty(accel_buffer)
                continue
            end
            
        else
            continue;
        end
        
        
        i = i + 1;
            
        % storing time samples

        t(i) = cmdTime;

        % storing gyro and acc reads
            
        gyroStore(i) = gyro_buffer(3);
        accStore(i) = accel_buffer(1);
            
        %filtering the gyroscope and accelerometer reads
            
        cutoff_freqL = (10)* 2.05;
        cutoff_freqH = (10)* 0.9;

        
        %% filtering the gyroscope and accelerometer reads
        
        %low pass filter 
        tcL = 1 / (2*pi*cutoff_freqL); % time constant
        smooth_factL = 1 - exp(-samplingTime / tcL); % smoothing factor
        lpf(1) = smooth_factL * gyroStore(1);
        lpf(i) = (smooth_factL * gyroStore(i) ) + ( 0.75* (1 - smooth_factL) * lpf(i-1) ); %output signal

        %high pass filter 
        tcH = 1 / (2*pi*cutoff_freqH);  % time constant
        smooth_factH = 1 - exp(-samplingTime / tcH); % smoothing factor
        hpf(1) = accStore(1);
        hpf(i) = (smooth_factH * accStore(i) ) + ( 0.25* (1 - smooth_factH) * hpf(i-1) ); %output signal

        % robot angular position
        dthG = gyroStore(i) * (t(i) - t(i-1))/(1000);      
        dth = lpf(i) * (t(i) - t(i-1))/(1000);      
        theta(i) = theta(i-1) + dth;

        if (theta(i)< -pi)
            theta(i) = theta(i) + 2 * pi;
        elseif (theta(i)> pi)
            theta(i) = theta(i) - 2 * pi;
        end
 
        % robot position
        Ax(i) = hpf(i);
        Ar(i) = abs(Ax(i)) ;
        Vr(i) = 2.1 * (Ar(i)*(t(i) - t(i-1))/(1000));
            
        Pr(i) = Vr(i);
        PosX(i) = PosX(i-1) + Pr(i) * cos(theta(i));
        disp(PosX(i));
        PosY(i) = PosY(i-1) + Pr(i) * sin(theta(i));
        disp(PosY(i));
        
        % plot in real-time
        x = get(line, 'xData');
        y  = get(line, 'yData');
               
        %---------------------------------------------------------------------------------
        x = [x, PosX(i)];   % change n to any variable you want plotted
        y = [y, PosY(i)]; % change m to any variable you want plotted
        %---------------------------------------------------------------------------------
        
        set(line, 'xData', x, 'yData', y);
        
        pause(0.1)

    end
    
        %% Task 5 - plotting the filtered and unfiltered readings (subtask a & b)

        %plotting gyroscope reads
        figure(1)
        plot(t, gyroStore, 'r');
        alpha_lpf = 0.0001        
        cutoff_freqL = (1 / samplingTime) * alpha_lpf;
        lpf = lowPassFilter(gyroStore, samplingTime, cutoff_freqL)
        hold on;
        plot(t, lpf, 'g');
        xlim([0 t(size(t,2))]);
        ylim([-0.5, 0.5]);
        grid on;
        title('Gyroscope Reads before and after filteration');
        legend('without filter','with low pass filter','Location',"northeast");
        
        %plotting accelerometer reads
        
        figure(2)        
        plot(t, accStore, 'r');
        hold on;
        alpha_hpf = 2
        cutoff_freqH = (1 / samplingTime) * alpha_hpf;
        hpf = highPassFilter(accStore, samplingTime, cutoff_freqH)
        plot(t, hpf, 'g');
        xlim([0 t(size(t,2))]);
        ylim([-0.5, 0.5]);
        grid on;
        xlabel('Time in seconds');
        ylabel('Acceleration');
        title('Accelerometer Reads before and after filteration');
        legend('without filter','with high pass filter','Location',"northeast");   
    
        %---------------------------------------------------------------------------------   

    
    % stop simulation
    [~]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    pause(5);

    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
    
    
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!



%% Filters Implementation   

%  sampling time (dt); time constant (tc = R*C) 

function lpf_op = lowPassFilter(lpf_ip, dt, cutoff_freq)
    tc = 1 / (2*pi*cutoff_freq); %time constant
    smooth_fact = dt / (tc + dt); %smoothing factor
    lpf_op(1) = smooth_fact * lpf_ip(1);
    
    for i = 2 : size(lpf_ip, 2)
        lpf_op(i) = (smooth_fact * lpf_ip(i) ) + ( (1 - smooth_fact) * lpf_op(i-1) );
    end
    return 
end

function hpf_op = highPassFilter(hpf_ip, dt, cutoff_freqH)
    tc = 1 / (2*pi*cutoff_freqH); %time constant
    smooth_factH = tc / (tc + dt); %smoothing factor
    hpf_op(1) = hpf_ip(1);
    
    for i = 2 : size(hpf_ip, 2)
        hpf_op(i) = (smooth_factH * hpf_op(i-1) ) + (smooth_factH * (hpf_ip(i) - hpf_ip(i-1)) );
    end
    return 
end


