% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)
function assignment_01 (serPort)

    % constants
    c_FastFwdVel = 0.2;
    c_SlowFwdVel = 0.1;
    c_VerySlowFwdVel = 0.05;
    c_BackOffVel = 0.05;
    c_BackOffDist = -0.1; % meters %

    % The flag is used to indicate if the obstable is seen.
    found_box = false

    % loop variables
    wallSensor = false
    bumped = false
    p_bRight = false
    p_bCenter = false
    p_bLeft = false
    p_ang = 0

    % Start robot moving: go straight
    SetFwdVelAngVelCreate (serPort, c_FastFwdVel, 0.0)

    % Enter main loop
    while (true)

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
            found_box = true

            p_ang = bumpReact (serPort, wallSensor, bRight, bCenter, bLeft)

            % turning is done, let the robot go straight.
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0)
        else

            if (found_box)
                if (wallSensor)
                    % a minor optimization using Wall Sensor
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0)
                else
                    turnAngle (serPort, 0.1, (-0.5) * p_ang);
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0)
                end
            end

%            % Step 2. If not hitting the wall, check if along the wall
%            if (false) % (wallSensor)
%                % if along the wall we go faster
%                print 'along the wall'
%                SetFwdVelAngVelCreate (serPort, c_FastFwdVel, 0.0)
%
%            else
%                if (found_box == true)
%                    % goodness, we need to found the box again
%                    print 'turning'
%                    SetFwdVelAngVelCreate (serPort, (-1.0) * c_FastFwdVel, 0.0)
%                    pause (0.1)
%
%                    do_right_turning (serPort)
%                else
%                    SetFwdVelAngVelCreate (serPort, c_FastFwdVel, 0.0)
%                end
%            end
        end

        % Briefly pause to avoid continuous loop iteration
        pause (0.1)
    end

    % Specify output parameter
    finalRad= v/w;

    % Stop robot motion
    SetFwdVelAngVelCreate (serPort, 0, 0)

    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    % fclose(serPort)
    % delete(serPort)
    % clear(serPort)
    % Don't use these if you call RoombaInit prior to the control program
end

% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated
function bumped= bumpCheckReact (serPort)

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight ...
     WheDropLeft WheDropCaster ...
     BumpFront] = BumpsWheelDropsSensorsRoomba (serPort);

    bumped = BumpRight || BumpLeft || BumpFront;

    % Halt forward motion and turn only if bumped
    if bumped

        % back off for a little bit
        travelDist (serPort, 0.05, -0.05);

        % Turn away from obstacle: always counter clock-wise (leverage the wall sensor)
        if ((BumpRight && BumpLeft) || BumpFront)
            ang = 15;
        elseif BumpRight
            ang = 15;
        elseif BumpLeft
            ang = 60;
        end

        % turn desired angle
        turnAngle (serPort, 0.1, ang);
    end
end

function t_ang= bumpReact (serPort, wall, right, center, left)

    % back off for a little bit
    travelDist (serPort, 0.05, -0.01);

    %if (wall) % consider wall sensor ?
    %    if ((right && left) || center)
    %        %turn counter-clockwise
    %        ang = 60
    %    else if (right)
    %        ang = 15
    %    else if (left)
    %        ang = 90
    %    end
    %else
        % Turn away from obstacle: always counter clock-wise (leverage the wall sensor)
        if ((right && left) || center)
            ang = 30;
        elseif right
            ang = 5;
        elseif left
            ang = 30;
        end
    %end

    % turn desired angle
    turnAngle (serPort, 0.1, ang);
    t_ang = ang
end

function wallSensor= wallCheck (serPort)

    [BumpRight BumpLeft BumpFront Wall virtWall CliffLft ...
     CliffRgt CliffFrntLft CliffFrntRgt LeftCurrOver ...
     RightCurrOver DirtL DirtR ButtonPlay ButtonAdv Dist ...
     Angle Volts Current Temp Charge Capacity pCharge]= ...
        AllSensorsReadRoomba (serPort)

    while true
        [BumpRight BumpLeft WheDropRight ...
         WheDropLeft WheDropCaster ...
         BumpFront] = BumpsWheelDropsSensorsRoomba (serPort);

        if (Wall)
            wallSensor= true;
        else
            wallSensor= false;
        end
    end
end

function do_right_turning (serPort)

    corner_met = false

    turnAngle (serPort, 0.1, -15);
    travelDist (serPort, 0.05, 0.10);

    while true
        % Check bump sensors (ignore wheel drop sensors)
        [BumpRight BumpLeft WheDropRight ...
         WheDropLeft WheDropCaster ...
         BumpFront] = BumpsWheelDropsSensorsRoomba (serPort);

        if BumpRight
            if corner_met == true
                break;
            else
                travelDist (serPort, 0.05, -0.10);
                turnAngle (serPort, 0.1, 15);
                travelDist (serPort, 0.05, 0.10);
            end
        else
            if corner_met == false
                corner_met == true
                turnAngle (serPort, 0.1, -30);
                travelDist (serPort, 0.05, 0.10);
            else
                turnAngle (serPort, 0.1, -30);
                travelDist (serPort, 0.05, 0.10);
            end
        end

        pause (0.1)
    end
end

function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)

    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)

    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end