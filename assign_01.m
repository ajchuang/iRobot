% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)
function assignment_01 (serPort)

    init_global ();

    % constants
    global c_FastFwdVel;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %
    global g_found_box;
    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;

    % loop variables
    wallSensor = false
    bumped = false
    p_bRight = false
    p_bCenter = false
    p_bLeft = false
    p_ang = 0

    % For the physical Create only, it is assumed that the function call
    % Calling RoombaInit is unnecessary if using the simulator.
    % serPort= RoombaInit(comPort) was done prior to running this program.

    % Start robot moving: go straight
    SetFwdVelAngVelCreate (serPort, c_FastFwdVel, 0.0)

    % Enter main loop
    while (true)

        % step 0: update last readings
        if (g_found_box)
            update_moving_stats (serPort);

            % check if mission completed
            if (checkMovingStats ())
                break;
            end
        end

        % Step 1. check if hitting the wall
        % Check for and react to bump sensor readings
        [bRight bLeft bCenter wallSensor virtWall CliffLft ...
         CliffRgt CliffFrntLft CliffFrntRgt LeftCurrOver ...
         RightCurrOver DirtL DirtR ButtonPlay ButtonAdv Dist ...
         Angle Volts Current Temp Charge Capacity pCharge]= ...
            AllSensorsReadRoomba (serPort)

        bumped = bRight | bCenter | bLeft;

        % If obstacle was hit reset distance and angle recorders
        if (bumped)

            print 'bump into the wall'

            if (g_found_box == false)
                DistanceSensorRoomba (serPort)
                AngleSensorRoomba (serPort)
            end

            g_found_box = true

            % remember the last angle we turn
            p_ang = bumpReact (serPort, wallSensor, bRight, bCenter, bLeft)

            % turning is done, let the robot go straight.
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0)
        else

            if (g_found_box)
                if (wallSensor)
                    % a minor optimization using Wall Sensor
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0)
                else
                    turnAngle (serPort, 0.1, (-1.0) * p_ang);
                    update_moving_stats (serPort);
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0)
                end
            end
        end

        % Briefly pause to avoid continuous loop iteration
        pause (0.1)
    end

    % Specify output parameter
    finalRad= 0.0;

    % Stop robot motion
    SetFwdVelAngVelCreate (serPort, 0, 0)

    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    % fclose(serPort)
    % delete(serPort)
    % clear(serPort)
    % Don't use these if you call RoombaInit prior to the control program
end

% init all global variables
function init_global ()

    % constants
    global c_FastFwdVel;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %
    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global g_found_box;
    global g_total_dist;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;

    % constants
    c_FastFwdVel = 0.2;
    c_SlowFwdVel = 0.1;
    c_VerySlowFwdVel = 0.05;
    c_BackOffVel = 0.05;
    c_BackOffDist = -0.03;      % meters %
    c_LeftTurnAngle = 30;
    c_RightTurnAngle = 5;
    c_CenterTurnAngle = 30;

    % The flag is used to indicate if the obstable is seen.
    g_found_box = false;
    g_total_dist = 0;
    g_total_x_dist = 0.0;
    g_total_y_dist = 0.0;
    g_total_angle = 0.0;
end

% init all global variables
function t_ang= bumpReact (serPort, wall, right, center, left)

    % constants
    global c_FastFwdVel;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %
    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global g_found_box;

    % back off for a little bit
    travelDist (serPort, c_BackOffVel, c_BackOffDist);
    update_moving_stats (serPort);

    % Turn away from obstacle: always counter clock-wise (leverage the wall sensor)
    if ((right && left) || center)
        ang = c_CenterTurnAngle;
    elseif right
        ang = c_RightTurnAngle;
    elseif left
        ang = c_LeftTurnAngle;
    end

    % turn desired angle
    turnAngle (serPort, 0.1, ang);
    update_moving_stats (serPort);
    t_ang = ang
end

% This is buggy -
function update_moving_stats (serPort)

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;

    dist = DistanceSensorRoomba (serPort)
    g_total_dist = g_total_dist + dist;
    g_total_angle = g_total_angle + AngleSensorRoomba (serPort);
    g_total_x_dist = g_total_x_dist + dist * cos (g_total_angle);
    g_total_y_dist = g_total_y_dist + dist * tan (g_total_angle);
end

function isDone= checkMovingStats ()

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;

    if (g_total_dist > 1.0 && g_total_x_dist <= 0.3 && g_total_y_dist <= 0.3)
        isDone = true;
    else
        isDone = false;
    end
end
