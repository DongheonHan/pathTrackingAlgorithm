% ORED LAB | DRV_MODEL_TEST | Version 1 2021-09-03

%% Clear Variables & Setup the timer
close all
clear, clc
tic                                 % Timer Starts
delayed_signal = 1;                 % Boolean
TIME_STEP_MS = 0.1;                 % 0.1sec (=10Hz)
outputVarArray = NaN(100000,2);

%% Create CAR_DRIVING_MODEL & CAR_AUTO_STEER Object
car_model = CAR_DRIVING_MODEL;
auto_steer = CAR_AUTO_STEER;

%% Create QUEUE Object
% Sensor Data
delayed_snr_PosX = QUEUE(100);     delayed_snr_PosY = QUEUE(100);
delayed_snr_Head = QUEUE(100);     delayed_snr_Speed = QUEUE(100);

% Actuator Data
delayed_act_Steer = QUEUE(100);
delayed_act_Speed = QUEUE(100);

%% Delay - CHANGE this variables
delay_snr_PosX = 0.0;   delay_snr_PosY = 0.0;
delay_snr_Head = 0.0;   delay_snr_Speed = 0.0;

delay_act_Steer = 0.1;
delay_act_Speed = 0.1;

%% setup the variables of car_model & auto_steer ”
car_model = car_model.setup();
auto_steer = auto_steer.setup();
carDrivingTest = 2; 
stopStartIndex = 1;

%% Arduino infinite loop
while(1)
    % Console Input
    if (stopStartIndex == 1)
        auto_steer = auto_steer.startControl();
    end

    dt = .1;
    
    % CAR_DRIVING_MODEL
    test_what = 2;
    
    % Car Driving Model Test
    if(1 == carDrivingTest)
        speed = 1.0;
        steer = deg2rad(30.0);
        car_model = car_model.update_model(speed, steer, dt);
        fprintf('Result: Cx: %d, Cy: %d, Head: %d \n', car_model.Cx, car_model.Cy, car_model.Head)               % print/save the values
    
    % Auto Steer Test
    elseif(2 == carDrivingTest)
        % driving state - sensor data
        car_posx = 0.0; car_posy = 0.0; car_head = 0.0; car_speed = 0.0;
        % driving command - actuator data
        cmd_steer = 0.0; cmd_speed = 0.0;
        
        if(delayed_signal)
            delayed_snr_PosX = delayed_snr_PosX.push(car_model.Cx);
            delayed_snr_PosY = delayed_snr_PosY.push(car_model.Cy);
            delayed_snr_Head = delayed_snr_Head.push(car_model.Head);
            delayed_snr_Speed = delayed_snr_Speed.push(car_model.SpeedMs);
            
            if((delayed_snr_PosX.getCount() - 1)*TIME_STEP_MS >= delay_snr_PosX)
                [delayed_snr_PosX, car_posx] = delayed_snr_PosX.pop();
            end
            
            if((delayed_snr_PosY.getCount() - 1)*TIME_STEP_MS >= delay_snr_PosY)
                [delayed_snr_PosY, car_posy] = delayed_snr_PosY.pop();
            end
            
            if((delayed_snr_Head.getCount() - 1)*TIME_STEP_MS >= delay_snr_Head)
                [delayed_snr_Head, car_head] = delayed_snr_Head.pop();
            end
            
            if((delayed_snr_Speed.getCount() - 1)*TIME_STEP_MS >= delay_snr_Speed)
                [delayed_snr_Speed, car_speed] = delayed_snr_Speed.pop();
            end
            
        else
            car_posx    = car_model.Cx;
            car_posy    = car_model.Cy;
            car_head    = car_model.Head;
            car_speed   = car_model.SpeedMs;
        end
        
        % 1. Update controller using current car driving state
        auto_steer = auto_steer.update(car_posx, car_posy, car_head, car_speed);
        
        if (delayed_signal)
            delayed_act_Steer = delayed_act_Steer.push(auto_steer.outWheelRad_FC);
            delayed_act_Speed = delayed_act_Speed.push(auto_steer.outSpeedMs_F);
            
            if((delayed_act_Steer.getCount() - 1)  * TIME_STEP_MS >= delay_act_Steer)
                [delayed_act_Steer, cmd_steer] = delayed_act_Steer.pop();
            end
            
            if((delayed_act_Speed.getCount() - 1)  * TIME_STEP_MS >= delay_act_Speed)
                [delayed_act_Speed, cmd_speed] = delayed_act_Speed.pop();
            end
        else
            cmd_steer  = auto_steer.outWheelRad_FC;
            cmd_speed  = auto_steer.outSpeedMs_F;
        end
        
        % 2. Update car model using car driving command
        car_model = car_model.update_model(cmd_speed, cmd_steer, dt);
        fprintf('PATH -  Cx:%d, Cy: %d, deg: %d\n', car_posx, car_posy, rad2deg(car_head))
        %fprintf('Waypoint Position: %d, %d\n\n', auto_steer.currWP_EndUtmX, auto_steer.currWP_EndUtmY)
        
        if (stopStartIndex > 1)
            if (auto_steer.currId_globalVar == 1)
                fprintf('Cycles Done \n')
                break
            end
        end
        outputVarArray(stopStartIndex,1) = car_posx;
        outputVarArray(stopStartIndex,2) = car_posy;
        stopStartIndex = stopStartIndex + 1; 
    else                                    % optional statement
        fprintf('invalid input, terminating the loop...\n')
        break
    end
end

%% Plot the results
figure(1)
plot(auto_steer.WayPointX, auto_steer.WayPointY, 'o')
xlim([-5,30])
ylim([-15,20])
hold on
plot(outputVarArray(:,1), outputVarArray(:,2), '.-')
title('ORED Lab Tractor Path Driving')
legend('Waypoints','Tractor''s Path')
grid on
hold off 

%% Save/Write Tractor_Path as a .mat file
isNaNIndx = find(isnan(outputVarArray(:,1)));
Tractor_Path = zeros(isNaNIndx(1)-1,2);
for i=1:1:isNaNIndx(1)-1
    Tractor_Path(i,1) = outputVarArray(i,1);
    Tractor_Path(i,2) = outputVarArray(i,2);
end
save('Tractor_Path_xy.mat', 'Tractor_Path')

%% 3D Simulation
load('Tractor_Path_xy.mat')

scenario = drivingScenario('SampleTime',0.05);
roadcenters1 = [auto_steer.WayPointX(1,1) auto_steer.WayPointY(1,1);...
    auto_steer.WayPointX(1,2) auto_steer.WayPointY(1,2)];
roadcenters2 = [auto_steer.WayPointX(1,2) auto_steer.WayPointY(1,2);...
    auto_steer.WayPointX(1,3) auto_steer.WayPointY(1,3)];
roadcenters3 = [auto_steer.WayPointX(1,3) auto_steer.WayPointY(1,3);...
    auto_steer.WayPointX(1,4) auto_steer.WayPointY(1,4)];
roadcenters4 = [auto_steer.WayPointX(1,4) auto_steer.WayPointY(1,4);...
    auto_steer.WayPointX(1,1) auto_steer.WayPointY(1,1)];

lspec = lanespec(1);
road(scenario,roadcenters1,'Lanes',lspec);
road(scenario,roadcenters2,'Lanes',lspec);
road(scenario,roadcenters3,'Lanes',lspec);
road(scenario,roadcenters4,'Lanes',lspec);

v = vehicle(scenario,'ClassID',1,'Length',2.3,'Width',1.3,'Height',1.6);

waypoints = Tractor_Path;
speeds = 5;

trajectory(v,waypoints,speeds)

plot(scenario,'Waypoints','on','RoadCenters','on')
view(-40, 40)

chasePlot(v)

while advance(scenario)
    pause(0.1)
end