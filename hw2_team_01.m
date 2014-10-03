%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 2
%
% Team number: 1
% Team leader: Jen-Chieh Huang (jh3478)
% Team members: Sze wun wong (sw2955)
%               Duo Chen (dc3026)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Input:
%       serPort - Serial port object, used for communicating over bluetooth
%
%   Output:
%       finalRad - Double, final turning radius of the Create (m)
%   
%   Thoughts:
%       States: ALONG_THE_LINE
%               CIRCUMVENT
%               COMPLETE
%  
function finalRad= hw2_team_01 (serPort)

    % constants
    global c_MacBook;
    global c_PortName;
    global c_TestingOn;

    global c_SimMode;
    global c_LoopInteval;
    global c_FastFwdVel;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_AfterBumpFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %

    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global c_TurnSpeed;
    global c_TurnRadius;

    global g_found_box;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;

    init_global ();

    % loop variables
    wallSensor  = false;
    bumped      = false;
    p_bRight    = false;
    p_bCenter   = false;
    p_bLeft     = false;
    p_ang       = 0;

    % For the physical Create only, it is assumed that the function call
    % Calling RoombaInit is unnecessary if using the simulator.
    if (c_SimMode == false)
        if (c_MacBook)
            serPort = RoombaInit_mac (c_PortName);
        else
            serPort = RoombaInit (c_PortName);
        end
    end

    % Start robot moving: go straight
    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % TESTING FUNC : AUTO-BUMPING TESTING FUNCTION                             %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (c_SimMode == false)
        % hit the wall and stop
        waitBump (serPort);
        BeepRoomba (serPort);
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Enter main loop: control cycle is about 2.3 (sec)
    while (true)

        % turn off the Roomba lights
        SetLEDsRoomba (serPort, 0, 0, 1);

        % step 0: update last readings
        update_moving_stats (serPort);

        % check if mission completed
        if (checkMovingStats ())
            display ('Found the end point - Stop!')
            break;
        end
        
        % check if we should leave the obstacle.
        if (checkExitObstacle (serPort) == true)
            display ('!!! unable to reach the goal - sorry !!!')
            break;
        end
        
        % Step 1. check if hitting the wall
        % Check for and react to bump sensor readings
        [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);

        if (isnan (bRight) || isnan (bCenter) || isnan (bLeft))
            display ('So bad - I dont know whats that');
            continue;
        else
            bumped = bRight | bCenter | bLeft;
        end

        % If obstacle was hit reset distance and angle recorders
        if (bumped)

            display ('bump into the wall');

            if (g_found_box == false)
                % reset moving stats
                DistanceSensorRoomba (serPort);
                AngleSensorRoomba (serPort);
                
                % set the fox is found
                g_found_box = true;
                
                reset_bumping_moving_status ();
            end

            % remember the last angle we turn
            p_ang = bumpReact (serPort, bRight, bCenter, bLeft);

            % turning is done, let the robot go straight.
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
        else
            if (g_found_box)

                % Optimization: read as needed
                wallSensor = WallSensorReadRoomba (serPort);

                if (isnan (wallSensor))
                    display ('!!! Bad COM - retrying !!!');
                    continue;
                elseif (wallSensor)
                    display ('wall sensor activated');

                    % a minor optimization using Wall Sensor
                    BeepRoomba (serPort);
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
                else
                    display ('doing differntial turning.');
                    find_wall (serPort);
                end
            end
        end

        % Briefly pause to avoid continuous loop iteration
        pause (c_LoopInteval);
    end

    % Stop robot motion
    turnAngle (serPort, c_TurnSpeed, 360);
    SetFwdVelAngVelCreate (serPort, 0, 0);

    BeepRoomba (serPort);
    BeepRoomba (serPort);

    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    if c_SimMode == false
        fclose (serPort);
        delete (serPort);
        clear  serPort;
    end

    % Specify output parameter
    finalRad = g_total_angle;
end

% TODO
% there is a hard-coded constant
% Corner case: need to be handled.
function neverEnd= checkExitObstacle (serPort)

    global g_found_box;
    global c_MaxToleranceRadius;
    global g_total_x_dist;
    global g_contact_x_dist;
    global g_total_y_dist;
    global g_contact_y_dist;
    global g_total_angle;
    global c_TurnSpeed;
    global c_SlowFwdVel;
    global g_goal_dist;

    % default value of neverEnd is false
    neverEnd = false;

    % check if the state changes --> hard coded, stupid function.
    % back to m_line again -
    % check if the state changes --> hard coded, stupid function.
    if (g_found_box == true && is_mline () == true)
        % back to m_line again -
        
        d = sqrt ((g_total_x_dist - g_contact_x_dist)^2 + (g_total_y_dist - g_contact_y_dist)^2);
        
        if (d >= 0.20 && abs(g_total_y_dist) < 0.1)
            display ('Leaving the box');
            
            % reset bumping memories
            g_found_box = false;
            reset_bumping_moving_status ();
            
            % stop and turn
            turnDeg = g_total_angle * (-1) * 180.0 / pi;
            SetFwdVelAngVelCreate (serPort, 0.0, 0);
            turnAngle (serPort, c_TurnSpeed, turnDeg);
            update_moving_stats (serPort);
            
            % keep going
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0);
            return;
        end
    end 
end

% a new state: trying to find the wall again.
function find_wall (serPort)
    
    global c_SlowFwdVel;
    global c_TurnRadius;
    global c_LoopInteval;
    global c_TurnSpeed;
    
    display ('doing differntial turning.');
                    
    %SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
    travelDist (serPort, c_SlowFwdVel, 0.05);
    turnAngle (serPort, c_TurnSpeed, 5);
    update_moving_stats (serPort);
    
    % start to turn
    display ('start to turn');
    SetFwdVelRadiusRoomba (serPort, c_TurnSpeed, c_TurnRadius);
    
    while (true)
        update_moving_stats (serPort);
        
        [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);
        wallSensor = WallSensorReadRoomba (serPort);

        if (isnan (bRight) || isnan (bCenter) || isnan (bLeft) || isnan (wallSensor))
            display ('So bad - I dont know whats that');
            continue;
        else
            bumped = bRight | bCenter | bLeft;
        end

        % If obstacle was hit reset distance and angle recorders
        if (bumped)
            display ('The wall is found ! - bump sensor');
            return;
        else 
            if (wallSensor)
                % wall sensor activated: go straight
                display ('The wall is found ! - wall sensor');
                SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
                return;
            end
        end
        
        pause (c_LoopInteval);
    end
end

% init all global variables
function init_global ()

    % constants
    global c_SimMode;
    global c_MacBook;
    global c_TestingOn;
    global c_PortName;
    global c_LoopInteval;
    global c_FastFwdVel;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %
    global c_TurnSpeed;
    global c_TurnRadius;
    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global c_MaxToleranceRadius;

    global g_goal_dist;
    global g_found_box;
    global g_total_dist;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_contact_x_dist;
    global g_contact_y_dist;

    % constants
    c_SimMode           = true;
    c_MacBook           = true;

    c_FastFwdVel        = 0.05;
    c_TurnRadius        = -0.20;

    if c_SimMode
        c_SlowFwdVel    = 0.3;
        c_TurnSpeed     = 0.2;
        c_LoopInteval   = 0.01;
    else
        if c_MacBook
            % machine dependent params
            c_PortName      = 'ElementSerial-ElementSe';
            c_TestingOn     = true;
            c_SlowFwdVel    = 0.025;
            c_TurnSpeed     = 0.025;
            c_LoopInteval   = 0.001;
        else
            % machine dependent params
            c_PortName      = 4;
            c_TestingOn     = true;
            c_SlowFwdVel    = 0.05;
            c_TurnSpeed     = 0.05;
            c_LoopInteval   = 0.1;
        end
    end

    c_AfterBumpFwdVel   = 0.075;
    c_VerySlowFwdVel    = 0.05;

    c_BackOffVel        = 0.025;
    c_BackOffDist       = -0.01;

    c_LeftTurnAngle     = 60;
    c_RightTurnAngle    = 15;
    c_CenterTurnAngle   = 45;

    c_MaxToleranceRadius = 0.15; % meters %

    % The flag is used to indicate if the obstable is seen.
    g_goal_dist         = 3.0;
    g_found_box         = false;
    g_total_dist        = 0;
    g_total_x_dist      = 0.0;
    g_total_y_dist      = 0.0;
    g_total_angle       = 0.0;
    g_contact_x_dist    = 0.0;
    g_contact_y_dist    = 0.0;
end

% init all global variables
function t_ang= bumpReact (serPort, right, center, left)

    % constants
    global c_BackOffVel;
    global c_BackOffDist; % meters %
    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global c_TurnSpeed;

    % back off for a little bit
    travelDist (serPort, c_BackOffVel, c_BackOffDist);
    update_moving_stats (serPort);

    % Turn away from obstacle: always counter clock-wise (leverage the wall sensor)
    if ((right && left) || center)
        display ('bumpReact: center')
        ang = c_CenterTurnAngle;
        SetLEDsRoomba (serPort, 3, 50, 50);
    elseif right
        display ('bumpReact: right')
        ang = c_RightTurnAngle;
        SetLEDsRoomba (serPort, 2, 0, 50);
    elseif left
        display ('bumpReact: left')
        ang = c_LeftTurnAngle;
        SetLEDsRoomba (serPort, 1, 100, 50);
    end

    % turn desired angle
    turnAngle (serPort, c_TurnSpeed, ang);
    update_moving_stats (serPort);
    t_ang = ang;
end

% TODO
% there is a hard-coded constant
function b_is_mline= is_mline ()
    global g_total_y_dist;
    global c_MaxToleranceRadius;
    
    b_is_mline = false;
    
    if (abs (g_total_y_dist) < 0.05)
        display ('m_line is found');
        b_is_mline = true;
        return;
    end
    
end

function reset_bumping_moving_status ()

    global g_total_x_dist_after_bump;
    global g_total_y_dist_after_bump;
    global g_contact_x_dist;
    global g_contact_y_dist;
    global g_contact_x;
    global g_contact_y;
    
    g_total_x_dist_after_bump = 0.0;
    g_total_y_dist_after_bump = 0.0;
    
    % remember the current bump point.
    g_contact_x_dist    = g_total_x_dist;
    g_contact_y_dist    = g_total_y_dist;
end

% This is buggy -
function update_moving_stats (serPort)

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;
    global g_total_x_dist_after_bump;
    global g_total_y_dist_after_bump;

    dist = DistanceSensorRoomba (serPort);
    angle = AngleSensorRoomba (serPort);
    
    if (isnan (dist) | isnan (angle))
        display ('!!! Bad Comm !!!');
        return;
    end

    g_total_dist = g_total_dist + dist;
    g_total_angle = g_total_angle + angle;
    g_total_x_dist = g_total_x_dist + dist * cos (g_total_angle);
    g_total_y_dist = g_total_y_dist + dist * sin (g_total_angle);
    
    g_total_x_dist_after_bump = g_total_x_dist_after_bump + dist * cos (g_total_angle);
    g_total_y_dist_after_bump = g_total_y_dist_after_bump + dist * sin (g_total_angle);
     
end

function isDone= checkMovingStats ()

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;
    global c_MaxToleranceRadius;
    global g_goal_dist;

    radius = sqrt ((g_total_x_dist - g_goal_dist) ^ 2 + g_total_y_dist ^ 2);

    display (sprintf ('current radius = %f', radius));
    display (sprintf ('current g_total_x_dist = %f, g_total_y_dist = %f', g_total_dist, g_total_y_dist));

    if (radius < c_MaxToleranceRadius)
        isDone = true;
    else
        isDone = false;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @lfred: a small trick to stop the robot right after the wall is hit
function waitBump (serPort)
    try
        set (serPort,'timeout',.01);

        %Flush buffer
        N = serPort.BytesAvailable();

        while (N ~= 0)
            fread (serPort, N);
            N = serPort.BytesAvailable ();
        end
    catch
    end

    %% Get (142) Wall Reading(8) data fields
    fwrite (serPort, [158 5]);
end
