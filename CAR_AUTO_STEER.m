% ORED LAB | DRV_MODEL_TEST | Version 1 2021-09-03

classdef CAR_AUTO_STEER
    properties
        %% Structure variables
        IS_PATH_LOOP;       % Path loop enabled
        IS_DTARGET_ENABLED; % Dynamic target enabled
        
        CTRL_CENTER_OFFSET; % Control center offset from vehicle position (m)
        
        TARGET_DIST_LONG;   % LookAhead targeting distance (Long Range, m)
        TARGET_DIST_SHORT;  % LookAhead targeting distance (Short Range, m)
        TARGET_ARR_RANGE;   % When car is in this range, arrival event on (m) (must be larger than TARGET_DIST_SHORT / 2.0)
        
        HEADERR_DISTCTRL_ON_RAD;     % if heading error is smaller than this value, apply distance control
        DISTERR_DISTCTRL_ON_M;       % if distance error is smaller than this value, apply distance control
        HEADERR_DTARGETCTRL_ON_RAD;  % if heading error is smaller than this value, apply dynamic target control too
        
        GAIN_HEAD_CTRL;     % [CTRL_HEAD] Heading Control Gain (0.0 ~ 1.0)
        GAIN_DIST_CTRL;     % [CTRL_DIST] Distance Control Gain (0.0 ~ 1.0)
        GAIN_DTARGET_CTRL;  % [CTRL_HEAD] Dynamic Target Control Gain
        
        STEER_LOWPASS_ALPHA; % Low pass alpha for final steer value
        SPEED_LOWPASS_ALPHA; % Low pass alpha for final speed value
        
        currId_globalVar;
        
        %% Public Variables 1
        % [INPUT] WAY POINTS
        WayPointX = NaN(1,10);    % UTM-X (m)
        WayPointY = NaN(1,10);    % UTM-Y (m)
        WaySpeedKmh = NaN(1,10);  % Path Speed (km/h) (Speed[0] = Start->WayPoint[0], Speed[1] = WayPoint[0]->WayPoint[1])
        
        % Vehicle State
        vehicleHeadRad;   % current vehicle heading (rad)
        vehicleSpeedMs;   % current vehicle speed (m/s)
        vehiclePosUtmX;   % control centerX (m)
        vehiclePosUtmY;   % control centerY (m)
        
        % [OUTPUT] Vehicle Command
        outSpeedMs;        % vehicle command output (speed, m/s)
        outSpeedMs_F;      % filtered vehicle command output (speed, m/s)
        
        outWheelRad;       % vehicle command output (wheel angle, rad)
        outWheelRad_F;     % filtered vehicle command output (wheel angle, rad)
        outWheelRad_FC;    % clamped & filtered vehicle command output (wheel angle, rad)
        
        %% Public Variables 2
        % Controller Variables
        isEnabled;          % is controller activated?
        isDataReady;        % is waypoints are ready?
        isPathEnd;          % is path-travel ended?
        
        currHeadErrRad;         % current head error between vehicle-head and path-head (rad)
        currDistErrM;           % current dist error between vehicle-pos and path-line (m)
        currTargetDistShort;    % current target distance (short range, m)
        currTargetDistLong;     % current target distance (long range, m)
        currTargetArrivalRange; % dicision range whether the car is arrived current target (fixed, m)
        
        % Waypoints Variables
        currWP_Size;        % Current Way-point Count of waypoints on Map-Path
        currWP_ID;          % current Way-point ID (END Index of current PATH)
        currWP_ID_Prev;     % Previous Way-point ID (BEGIN Index of current PATH)
        
        currWP_BeginUtmX;  % current Way-point Begin UTMX(m)
        currWP_BeginUtmY;  % current Way-point Begin UTMY(m)
        currWP_EndUtmX;    % current Way-point End UTMX(m)
        currWP_EndUtmY;    % current Way-point End UTMY(m)
        currTargetUtmX;    % current Target-point UTMX(m)
        currTargetUtmY;    % current Target-point UTMY(m)
        
        vec_BeginEnd = NaN(1,2);       % vector (from: currWP_Begin, to: currWP_End) ==> necessary for calculating <DIST ERROR>
        vec_VehicleTarget = NaN(1,2);  % vector (from: vehiclePos, to: currTarget) ==> necessary for calculating <HEAD ERROR>
        
        g_ppx1;
        g_ppy1;
        g_ppx2; 
        g_ppy2;
        
    end
    
    methods
        % Reset the structure
        function obj = reset(obj)
            obj.IS_PATH_LOOP = 1;
            obj.IS_DTARGET_ENABLED = 0;
            
            obj.CTRL_CENTER_OFFSET  = 0.0;
            
            obj.TARGET_DIST_LONG    = 6.0;
            obj.TARGET_DIST_SHORT   = 3.0;
            obj.TARGET_ARR_RANGE    = 2.5;   % (larger than 3.0 / 2 = 1.5)
            
            obj.HEADERR_DISTCTRL_ON_RAD     = deg2rad(5.0);   % dist control when < 5deg
            obj.DISTERR_DISTCTRL_ON_M       = 1.0;                % dist control when < 1m
            obj.HEADERR_DTARGETCTRL_ON_RAD  = deg2rad(25.0);  % dtarget control when < 25deg
            
            obj.GAIN_HEAD_CTRL    = 1.0;
            obj.GAIN_DIST_CTRL    = 0.0;
            obj.GAIN_DTARGET_CTRL = 2.0;
            
            obj.STEER_LOWPASS_ALPHA = 0.1;
            obj.SPEED_LOWPASS_ALPHA = 0.1;
        end
        function obj = setup(obj)
            
            % Reset structure variables
            obj = reset(obj);
            
            for i = 1:1:10
                obj.WayPointX(i) = 0.0;
                obj.WayPointY(i) = 0.0;
                obj.WaySpeedKmh(i) = 0.0;
            end
            
            obj.vehicleHeadRad  = 0.0;
            obj.vehicleSpeedMs  = 0.0;
            obj.vehiclePosUtmX = 0.0;
            obj.vehiclePosUtmY = 0.0;
            obj.outSpeedMs = 0.0;
            obj.outWheelRad = 0.0;
            
            obj.outSpeedMs_F = 0.0;
            obj.outWheelRad_F = 0.0;
            obj.outWheelRad_FC = 0.0;
            
            obj.isEnabled = 0;
            obj.isDataReady = 0;
            obj.isPathEnd = 0;
            
            obj.currHeadErrRad = 0.0;
            obj.currDistErrM = 0.0;
            
            obj.currWP_Size = 0.0;
            obj.currWP_ID = 0.0;
            obj.currWP_ID_Prev = 0;
            obj.currWP_BeginUtmX  = 0.0;    obj.currWP_BeginUtmY  = 0.0;
            obj.currWP_EndUtmX    = 0.0;    obj.currWP_EndUtmY    = 0.0;
            obj.currTargetUtmX    = 0.0;    obj.currTargetUtmY    = 0.0;
            
            obj.vec_BeginEnd(1,1) = 0.0;
            obj.vec_BeginEnd(1,2) = 0.0;
            
            obj.vec_VehicleTarget(1,1)  = 0.0;
            obj.vec_VehicleTarget(1,2)  = 0.0;
            
            obj.currTargetDistShort     = obj.TARGET_DIST_SHORT;
            obj.currTargetDistLong      = obj.TARGET_DIST_LONG;
            obj.currTargetArrivalRange  = obj.TARGET_ARR_RANGE;
        end
        
        function obj = startControl(obj)
            obj = obj.setup_path();
            obj.isEnabled = 1;
            obj.isPathEnd = 0;
        end
        
        function obj = stopControl(obj)
            obj.isEnabled = 0;
            obj.isPathEnd = 1;
        end
        
        function obj = update(obj, car_pos_x, car_pos_y, car_head_rad, car_speed_ms)
            obj = obj.update_vehicle(car_pos_x, car_pos_y, car_head_rad, car_speed_ms);
            if (obj.isEnabled && obj.isDataReady)
                obj = obj.update_waypath();
                obj = obj.update_targetpoint();
                obj = obj.update_autosteer();
                obj = obj.update_autospeed();
                obj = obj.update_final_command(0); % engaging driving
            else
                update_final_command(1) % stop driving
            end
            
        end
    end
    
    % Private Method
    methods (Access = private)
        function obj = setup_path(obj)
            obj.currWP_Size     = 4;    % 4 waypoints in this example
            obj.currWP_ID       = 1;    % First waypoint index (0)
            obj.currWP_ID_Prev  = 0;   % Previous waypoint index (-1)
            
            %obj.WayPointX(1,1) = 5.0;     obj.WayPointY(1,1) = 0.0;       obj.WaySpeedKmh(1,1) = 2.0;
            %obj.WayPointX(1,2) = 15.0;    obj.WayPointY(1,2) = 10.0;      obj.WaySpeedKmh(1,2) = 2.0;
            %obj.WayPointX(1,3) = 25.0;    obj.WayPointY(1,3) = 0.0;       obj.WaySpeedKmh(1,3) = 2.0;
            %obj.WayPointX(1,4) = -15.0;   obj.WayPointY(1,4) = 10.0;      obj.WaySpeedKmh(1,4) = 2.0;
            
            obj.WayPointX(1,1) = 5.0;     obj.WayPointY(1,1) = 0.0;       obj.WaySpeedKmh(1,1) = 2.0;
            obj.WayPointX(1,2) = 15.0;    obj.WayPointY(1,2) = 10.0;      obj.WaySpeedKmh(1,2) = 2.0;
            obj.WayPointX(1,3) = 25.0;    obj.WayPointY(1,3) = 0.0;       obj.WaySpeedKmh(1,3) = 2.0;
            obj.WayPointX(1,4) = 15.0;    obj.WayPointY(1,4) = -10.0;     obj.WaySpeedKmh(1,4) = 2.0;
            
            %  When path has just setup, we will set vehicle-point as begin-point
            obj.currWP_BeginUtmX  = obj.vehiclePosUtmX;
            obj.currWP_BeginUtmY  = obj.vehiclePosUtmY;
            obj.currWP_EndUtmX    = obj.WayPointX(1,1);
            obj.currWP_EndUtmY    = obj.WayPointY(1,1);
            
            obj.isDataReady     = 1;
            
        end
        
        % (1) Update Vehicle Status
        function obj = update_vehicle(obj, car_pos_x, car_pos_y, car_head_rad, car_speed_ms)
            obj.vehiclePosUtmX = car_pos_x + obj.CTRL_CENTER_OFFSET * cos(car_head_rad); % Shift control-center by offset param
            obj.vehiclePosUtmY = car_pos_y + obj.CTRL_CENTER_OFFSET * sin(car_head_rad); % Shift control-center by offset param
            obj.vehicleHeadRad = car_head_rad;
            obj.vehicleSpeedMs = car_speed_ms;
        end
        
        % (2) Update Next Waypoint (if car has been arrived at current target)
        function obj = update_waypath(obj)
            % Check Car Arrival event at current Path-End-Position (check distance is smaller than target arrival range)
            bJumpToNextWaypoint = 0;
            dist_vehicle_to_end_point = sqrt((obj.currWP_EndUtmX - obj.vehiclePosUtmX)...
                *(obj.currWP_EndUtmX - obj.vehiclePosUtmX) + (obj.currWP_EndUtmY ...
                - obj.vehiclePosUtmY)*(obj.currWP_EndUtmY - obj.vehiclePosUtmY)); 
            
            if (dist_vehicle_to_end_point < obj.currTargetArrivalRange)
                bJumpToNextWaypoint = 1;
            end
            
            %  Jump to next waypoint and update contorol positions (JUST ONCE ON EVENT)
            
            if(bJumpToNextWaypoint)
                obj.currWP_ID_Prev = obj.currWP_ID;
                obj.currWP_ID = obj.currWP_ID + 1;

                if(obj.currWP_ID > obj.currWP_Size)
                    if(obj.IS_PATH_LOOP)
                        obj.currWP_ID = 1;
                    else
                        obj.currWP_ID = -1;
                        obj = stopControl();
                    end
                end

                % Update Waypoint Positions
                obj.currWP_BeginUtmX  = obj.WayPointX(obj.currWP_ID_Prev);
                obj.currWP_BeginUtmY  = obj.WayPointY(obj.currWP_ID_Prev);
                obj.currWP_EndUtmX    = obj.WayPointX(obj.currWP_ID);
                obj.currWP_EndUtmY    = obj.WayPointY(obj.currWP_ID);
                obj.currId_globalVar = obj.currWP_ID; %%%%%
            end
        end
        
        % (3) Find Target Point
        function obj = update_targetpoint(obj)
            % Update current path-line vector
            obj.vec_BeginEnd(1,1) = obj.currWP_EndUtmX - obj.currWP_BeginUtmX;
            obj.vec_BeginEnd(1,2) = obj.currWP_EndUtmY - obj.currWP_BeginUtmY;

            % Find Target Point (using intersection between vehicle-circle and path-line)
            Ox = obj.currWP_BeginUtmX;  % Line origin
            Oy = obj.currWP_BeginUtmY;  % Line origin
            Dx = obj.vec_BeginEnd(1,1);   % Line direction
            Dy = obj.vec_BeginEnd(1,2);   % Line direction
            Cx = obj.vehiclePosUtmX;    % Circle center
            Cy = obj.vehiclePosUtmY;    % Circle center
            Ex = obj.currWP_EndUtmX;    % Line end
            Ey = obj.currWP_EndUtmY;    % Line end
            
            % Check Long Range Circle Firstly
            radius = obj.currTargetDistLong;

            [Large_circle_Bool, ~, ~, ~, ~, ~, ~] = ...
                obj.IntersectLineCircle(Ox, Oy, Dx, Dy, Cx, Cy, radius);
            
            if(Large_circle_Bool) % Large circle intersects with path-line
                % Check Short Range Additionally
                radius = obj.currTargetDistShort;
                [small_circle_Bool, ppx1, ppy1, ppx2, ppy2, ~, ~] =...
                    obj.IntersectLineCircle(Ox, Oy, Dx, Dy, Cx, Cy, radius);
                
                if (small_circle_Bool == 1)
                    obj.g_ppx1 = ppx1;
                    obj.g_ppy1 = ppy1;
                    obj.g_ppx2 = ppx2;
                    obj.g_ppy2 = ppy2;              
                end
                
                
                if(small_circle_Bool) % Small circle intersects with path-line
                    % Path is on SHORT range (==> Targeting closest circle cross point)
                    [ttx, tty] = obj.FindClosestPointToEndWP(ppx1, ppy1, ppx2, ppy2, Ex, Ey);
                    obj.currTargetUtmX = ttx;
                    obj.currTargetUtmY = tty;
                else
                    %  Path is on LONG (not SHORT) range (==> Targeting closest circle cross point)
  
                    [tx, ty] = obj.FindClosestPointToEndWP(obj.g_ppx1, obj.g_ppy1, obj.g_ppx2, obj.g_ppy2, Ex, Ey);
                    obj.currTargetUtmX = tx;
                    obj.currTargetUtmY = ty;
                end
                % Vehicle is Far from current way-path (==> Targeting End-Waypoint)
            else
                obj.currTargetUtmX = obj.currWP_EndUtmX;
                obj.currTargetUtmY = obj.currWP_EndUtmY;
            end
            % Update target vector (Target point from Vehicle)
            obj.vec_VehicleTarget(1,1) = obj.currTargetUtmX - obj.vehiclePosUtmX;
            obj.vec_VehicleTarget(1,2) = obj.currTargetUtmY - obj.vehiclePosUtmY;
        end
        
        % (4) Find AutoSteer value
        function obj = update_autosteer(obj)
            % (4.1) Update Distance Error between [vec_BeginEnd] and [Vehicle Position]
            if(obj.currWP_ID > 0)
                
                obj.currDistErrM = DistFromPointToLine(obj, obj.vehiclePosUtmX,...
                    obj.vehiclePosUtmY,obj.currWP_BeginUtmX,...
                    obj.currWP_BeginUtmY,obj.currWP_EndUtmX,obj.currWP_EndUtmY);
                
            else
                obj.currDistErrM = 0.0;
            end
            
            % (4.2) Update Heading Error between [vec_VehicleTarget] and [Vehicle Heading]
            head_vec_VehicleTarget = atan2(obj.vec_VehicleTarget(1,2), obj.vec_VehicleTarget(1,1));
            obj.currHeadErrRad = CAR_DRIVING_MODEL.CLAMP_RADIANS(obj.vehicleHeadRad...
                - head_vec_VehicleTarget);

            head_err    = obj.currHeadErrRad;
            dist_err    = obj.currDistErrM;
            vehicle_speed   = obj.vehicleSpeedMs;
            
            % (4.3) Derive Head-error Control Value from Head-error
            head_err_ratio_inverse = 0.0; % static variable
            
            if (abs(head_err) <= obj.HEADERR_DTARGETCTRL_ON_RAD)
                head_err_ratio_inverse = (obj.HEADERR_DTARGETCTRL_ON_RAD - abs(head_err))...
                    / obj.HEADERR_DTARGETCTRL_ON_RAD;
            end
            
            % (4.4) Update Target Circle Range (using head-error and vehicle-speed)
            if (obj.IS_DTARGET_ENABLED)             % Dynamic Target Range %% always disabled
                obj.currTargetDistShort = obj.TARGET_DIST_SHORT * ...
                    (1.0 + head_err_ratio_inverse * obj.GAIN_DTARGET_CTRL * vehicle_speed);
                obj.currTargetDistLong = obj.TARGET_DIST_LONG * ...
                    (1.0 + head_err_ratio_inverse * obj.GAIN_DTARGET_CTRL * vehicle_speed);
                
            else                                    % Static Target Range
                obj.currTargetDistShort = obj.TARGET_DIST_SHORT;
                obj.currTargetDistLong = obj.TARGET_DIST_LONG;
            end
            
            % (4.5) Generate Distance Control Value (P-Control)
            if(abs(dist_err) <= obj.DISTERR_DISTCTRL_ON_M && abs(head_err) < obj.HEADERR_DISTCTRL_ON_RAD)
                dist_ctrl_value = - dist_err * obj.GAIN_DIST_CTRL;
            else
                dist_ctrl_value = 0.0;
            end
            
            % (4.6) Generate Heading Control Value (P-Control)
            head_ctrl_value = -head_err * obj.GAIN_HEAD_CTRL;
            
            % (4.7) Generate Steer Angle from two control values
            steer_cmd = head_ctrl_value + dist_ctrl_value;
            
            % (4.8) Generate Final Wheel Angle Output
            obj.outWheelRad     = steer_cmd; % Raw command
            obj.outWheelRad_F   = obj.outWheelRad_F + (obj.outWheelRad - ...
                obj.outWheelRad_F) * obj.STEER_LOWPASS_ALPHA; % Filtered command
            obj.outWheelRad_FC  = CAR_DRIVING_MODEL.CLAMP_RADIANS(obj.outWheelRad_F); % Clamped & Filtered command

        end
        
        % (5) Find Speed value
        function obj = update_autospeed(obj)
            if(~obj.isEnabled || ~obj.isDataReady || obj.isPathEnd || obj.currWP_ID < 0)
                obj.outSpeedMs = 0.0;
            else
                obj.outSpeedMs = obj.WaySpeedKmh(obj.currWP_ID)/ 3.6;
            end
            obj.outSpeedMs_F = obj.outSpeedMs_F + (obj.outSpeedMs - ...
                obj.outSpeedMs_F) * obj.SPEED_LOWPASS_ALPHA; % Filtered command
        end
        
        % (6) Final Limit Check
        function obj = update_final_command(obj, stop)
            if(stop)
                obj.outWheelRad = 0.0;
                obj.outWheelRad_F = 0.0;
                obj.outWheelRad_FC = 0.0;
                obj.outSpeedMs = 0.0;
                obj.outSpeedMs_F = 0.0;
            else
                steer_limit_rad = deg2rad(40.0);
                speed_limit_ms = 30.0 / 3.6;
                if(obj.outWheelRad_FC >= steer_limit_rad)
                    obj.outWheelRad_FC = steer_limit_rad;
                end
                if(obj.outWheelRad_FC <= -steer_limit_rad)
                    obj.outWheelRad_FC = -steer_limit_rad;
                end
                if(obj.outSpeedMs_F >= speed_limit_ms)
                    obj.outSpeedMs_F = speed_limit_ms;
                end
                if(obj.outSpeedMs_F <= 0.0)
                    obj.outSpeedMs_F = 0.0;
                end
            end
        end
        
        % UTILITY FUNCTIONS
        %  Find closest waypoint id using vehicle position
        function closestWaypoint = find_closest_wp(obj) 
            dist_min = 100000.0;
            dist_min_id = -1;
            
            for i = 1:1:obj.currWP_Size
                dist = sqrt((obj.WayPointX(i)-obj.vehiclePosUtmX)*(obj.WayPointX(i)-obj.vehiclePosUtmX) +...
                    (obj.WayPointY(i)-obj.vehiclePosUtmY)*(obj.WayPointY(i)-obj.vehiclePosUtmY));
                if(dist < dist_min)
                    dist_min = dist;
                    dist_min_id = i;
                end
            end
            
            if(dist_min_id >= 0)
                if(dist_min_id >= obj.currWP_Size-1)
                    closestWaypoint = 0;
                else
                    closestWaypoint = dist_min_id + 1;
                end
            else
                closestWaypoint = 0;
            end
        end
        
        function [intSecLCircle, point1x, point1y, point2x, point2y, param1, param2] ...
                = IntersectLineCircle(~, Ox, Oy, Dx, Dy, Cx, Cy, radius)
            
            dx = Ox - Cx;
            dy = Oy - Cy;
            
            a = Dx*Dx + Dy*Dy;
            b = dx*Dx + dy*Dy;
            c = dx*dx + dy*dy - radius * radius;
            
            t = NaN(2);
            
            disc = b * b - a * c;
           
            if (disc < 0.0)
                intSecLCircle = 0;  % return 0 only for this case
                point1x = 0;
                point1y = 0;
                point2x = 0;
                point2y = 0;
                param1 = 0;
                param2 = 0;
                return
            end
            
            sqrtDisc = sqrt(disc);
            
            if(abs(a) > 0.000000000001)
                invA = 1.0 / a;
                t(1) = (-b - sqrtDisc) * invA;
                t(2) = (-b + sqrtDisc) * invA;
            else
                intSecLCircle = 0;  % return 0 only for this case
                return
            end
            point1x = Ox + t(1) * Dx;   point1y = Oy + t(1) * Dy;
            point2x = Ox + t(2) * Dx;   point2y = Oy + t(2) * Dy;
            
            param1 = t(1); param2 = t(2);
            intSecLCircle = 1;
            return
        end
        
        % Find the more closer point to the END-WAYPOINT among above two intersection points
        function [targetx, targety] =  FindClosestPointToEndWP(~, point1x, point1y,...
                point2x, point2y, endwp_x, endwp_y)
            sqdist_p1_endwp = (point1x - endwp_x)*(point1x - endwp_x)...
                + (point1y - endwp_y)*(point1y - endwp_y);
            sqdist_p2_endwp = (point2x - endwp_x)*(point2x - endwp_x)...
                + (point2y - endwp_y)*(point2y - endwp_y);
            if (sqdist_p1_endwp < sqdist_p2_endwp)
                targetx = point1x;  targety = point1y;
            else
                targetx = point2x;  targety = point2y;
            end
        end
        
        function DstFrmPntTLn = DistFromPointToLine(~, x, y, x1, y1, x2, y2)
            
            % x,y: point, x1y1 - y1y2: line vector
            A = x - x1;  B = y - y1;  C = x2 - x1; 
            D = y2 - y1; E = x - x2;  F = y - y2;
            
            dot = A * C + B * D;
            cross = A * D - B * C;
            len_sq = C * C + D * D;
            param = -1;
            
            if(len_sq ~= 0)
                param = dot / len_sq;
            end
            
            if(param < 0)
                len_AB = sqrt(A * A + B * B);
                len_CD = sqrt(C * C + D * D);
                sin_theta = cross / (len_AB * len_CD);
                dist = len_AB * sin_theta;
                DstFrmPntTLn = dist;
                return
                
            elseif (param > 1)
                len_EF = sqrt(E * E + F * F);
                len_CD = sqrt(C * C + D * D);
                cross2 = E * D - F * C;
                sin_theta = cross2 / (len_EF * len_CD);
                dist = len_EF * sin_theta;
                DstFrmPntTLn = dist;
                return
            else
                xx = x1 + param * C;
                yy = y1 + param * D;
            end
            dx = x - xx;
            dy = y - yy;
            ret = sqrt(dx * dx + dy * dy);
            
            if (cross >= 0.0)
                DstFrmPntTLn = ret;
                return
            else
                DstFrmPntTLn = -ret;
                return
            end
        end
    end
end