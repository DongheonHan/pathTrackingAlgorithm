% ORED LAB | DRV_MODEL_TEST | Version 1 2021-09-03

classdef CAR_DRIVING_MODEL
    properties % Public Variables
        
        % Driving Input Variables
        SteerRd;  SpeedMs;
        
        % Driving Kinematic Variables (Tractor)
        Foffset;    % Front-wheel distance from center (m)
        Roffset;    % Rear-wheel distance from center (m)
        
        Cx; Cy;     % Center position (m)
        Fx; Fy;     % Front-wheel position (m)
        Rx, Ry;     % Rear-wheel position (m)
        Head;       % Body Head angle (rad)
        
        % Driving Kinematic Variables (Trailer)
        Joffset;    % Joint distance from center (m)
        Toffset;    % Trailer distance from Joint (m)
        
        Jx; Jy;     % Joint position (m)
        Tx; Ty;     % Trailer position (m)
        T_Head;     % Trailer Head angle (rad)
    end
    
    methods
        function obj = setup(obj)
            % Input Parameters
            obj.SteerRd = 0;    obj.SpeedMs = 0;
            
            % CAR Geometry
            obj.Foffset = 1.5;      obj.Roffset = 0.5;
            
            % Initial Value
            obj.Head = 0.0;                 obj.T_Head = 0.0;
            obj.Cx = 0.0;                   obj.Cy = 0.0;
            obj.Fx = obj.Cx + obj.Foffset;  obj.Fy = 0.0;
            obj.Rx = obj.Cx - obj.Roffset;  obj.Ry = 0.0;
            
            obj.Jx = obj.Cx - obj.Joffset;  obj.Jy = 0.0;
            obj.Tx = obj.Jx - obj.Toffset;  obj.Ty = 0.0;
        end
        
        function obj = update_model(obj, SPEED, STEER, DT)
            % Firstly, update input parameters
            obj.SteerRd = STEER;    obj.SpeedMs = SPEED;
            
            % (1) Derive Current Front/Rear wheel position from body center
            obj.Fx = obj.Cx + obj.Foffset * cos(obj.Head);
            obj.Fy = obj.Cy + obj.Foffset * sin(obj.Head);
            obj.Rx = obj.Cx - obj.Roffset * cos(obj.Head);
            obj.Ry = obj.Cy - obj.Roffset * sin(obj.Head);
            
            % (2) Update new Front/Rear wheel position using STEER and SPEED
            obj.Fx = obj.Fx + SPEED * DT * cos(obj.Head + STEER);
            obj.Fy = obj.Fy + SPEED * DT * sin(obj.Head + STEER);
            obj.Rx = obj.Rx + SPEED * DT * cos(obj.Head);
            obj.Ry = obj.Ry + SPEED * DT * sin(obj.Head);
            
            % (3) Derive new Body position from new Front/Rear wheel position
            obj.Head = CAR_DRIVING_MODEL.CLAMP_RADIANS(atan2(obj.Fy - obj.Ry, obj.Fx - obj.Rx));
            obj.Cx = obj.Rx + obj.Roffset * cos(obj.Head);
            obj.Cy = obj.Ry + obj.Roffset * sin(obj.Head);
            
            % (4) Derive new Joint position from new Body position
            obj.Jx = obj.Cx - obj.Joffset * cos(obj.Head);
            obj.Jy = obj.Cy - obj.Joffset * sin(obj.Head);
            
            % (5) Derive new Joint position from new Body position
            obj.T_Head = CAR_DRIVING_MODEL.CLAMP_RADIANS(atan2(obj.Jy - obj.Ty, obj.Jx - obj.Tx));
            obj.Tx = obj.Jx - obj.Toffset * cos(obj.T_Head);
            obj.Ty = obj.Jy - obj.Toffset * sin(obj.T_Head);
        end
    end

    methods (Static)
        
        % Adjust angle over TWO_PI
        function clamp_radians_return = CLAMP_RADIANS(r)
            if r > 0.0
                d = 2*pi;
            else
                d = -2*pi;
            end
            r = mod(r, d);
            
            if (abs(r) > pi)
                clamp_radians_return = r - d;
                return
            else
                clamp_radians_return = r;
                return
            end
        end
        
    end
end