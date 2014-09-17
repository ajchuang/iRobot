% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)
function assignment_01 (serPort)

    % constants
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
        serPort = RoombaInit_mac ('ElementSerial-ElementSe');
    end

    % Start robot moving: go straight
    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);

    % Enter main loop: control cycle is about 2.3 (sec)
    while (true)

        % turn off the Roomba lights
        SetLEDsRoomba (serPort, 0, 0, 1);

        % step 0: update last readings
        if (g_found_box)
            update_moving_stats (serPort);

            % check if mission completed
            if (checkMovingStats ())
                display ('Found the starting point - Stop!')
                break;
            end
        end

        % Step 1. check if hitting the wall
        % Check for and react to bump sensor readings
        [bRight bLeft bCenter wallSensor virtWall CliffLft ...
         CliffRgt CliffFrntLft CliffFrntRgt LeftCurrOver ...
         RightCurrOver DirtL DirtR ButtonPlay ButtonAdv Dist ...
         Angle Volts Current Temp Charge Capacity pCharge]= ...
            AllSensorsReadRoomba (serPort);

        bumped = bRight | bCenter | bLeft;

        % If obstacle was hit reset distance and angle recorders
        if (bumped)

            display ('bump into the wall');

            if (g_found_box == false)
                % reset moving stats
                DistanceSensorRoomba (serPort);
                AngleSensorRoomba (serPort);
            end

            % set the fox is found
            g_found_box = true;

            % remember the last angle we turn
            p_ang = bumpReact (serPort, wallSensor, bRight, bCenter, bLeft);

            % turning is done, let the robot go straight.
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
        else

            if (g_found_box)
                if (wallSensor)
                    display ('wall sensor activated - go straight');

                    % a minor optimization using Wall Sensor
                    BeepRoomba (serPort);
                    SetFwdVelAngVelCreate (serPort, c_FastFwdVel, 0.0);
                else
                    % need to find the wall again
                    turnAngle (serPort, c_TurnSpeed, (-0.67) * p_ang);
                    update_moving_stats (serPort);

                    % Running a little bit hurry
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
                end
            end
        end

        % Briefly pause to avoid continuous loop iteration
        pause (c_LoopInteval);
    end

    % Specify output parameter
    finalRad = 0.0;

    % Stop robot motion
    SetFwdVelAngVelCreate (serPort, 0, 0);

    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    if c_SimMode == false
        fclose (serPort);
        delete (serPort);
        clear  (serPort);
    end
    % Don't use these if you call RoombaInit prior to the control program
end

% init all global variables
function init_global ()

    % constants
    global c_SimMode;
    global c_LoopInteval;
    global c_FastFwdVel;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %
    global c_TurnSpeed;
    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global c_MaxToleranceRadius;

    global g_found_box;
    global g_total_dist;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;

    % constants
    c_SimMode           = false;
    c_LoopInteval       = 0.001;
    c_FastFwdVel        = 0.05;
    c_SlowFwdVel        = 0.025;
    c_AfterBumpFwdVel   = 0.075;
    c_VerySlowFwdVel    = 0.05;

    c_BackOffVel        = 0.025;
    c_BackOffDist       = -0.01;
    c_TurnSpeed         = 0.025;

    c_LeftTurnAngle     = 60;
    c_RightTurnAngle    = 20;
    c_CenterTurnAngle   = 45;

    c_MaxToleranceRadius = 0.3; % meters %

    % The flag is used to indicate if the obstable is seen.
    g_found_box         = false;
    g_total_dist        = 0;
    g_total_x_dist      = 0.0;
    g_total_y_dist      = 0.0;
    g_total_angle       = 0.0;
end

% init all global variables
function t_ang= bumpReact (serPort, wall, right, center, left)

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

% This is buggy -
function update_moving_stats (serPort)

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;

    dist = DistanceSensorRoomba (serPort);
    g_total_dist = g_total_dist + dist;
    g_total_angle = g_total_angle + AngleSensorRoomba (serPort);
    g_total_x_dist = g_total_x_dist + dist * cos (g_total_angle);
    g_total_y_dist = g_total_y_dist + dist * sin (g_total_angle);
end

function isDone= checkMovingStats ()

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;
    global c_MaxToleranceRadius;

    radius = sqrt (g_total_x_dist ^ 2 + g_total_y_dist ^ 2);

    display (sprintf ('current radius = %f', radius));
    display (sprintf ('current g_total_dist = %f', g_total_dist));

    if (g_total_dist > 0.5 && radius < c_MaxToleranceRadius)
        isDone = true;
    else
        isDone = false;
    end
end
