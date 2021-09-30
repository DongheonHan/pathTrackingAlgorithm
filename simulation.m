% ORED LAB | simulation 
% Author: Dongheon Han
% This code is made to record the results.
% Execute tractor_path_driving first to save the variables.
% Press "s" to start the animation
clc
close all
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

rc = input('input s (start) if recording is ready:  ', 's');
if("s"  == rc)
    while advance(scenario)
        pause(0.1)
    end
end
