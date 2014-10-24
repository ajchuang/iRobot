%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 3
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Configuration:
%       before you started, make sure the following things
%       0. Go to 'init_global'
%       1. If you are running simulator, set the 'c_SimMode = true'
%       2. If you are running real robot, 
%           set 'c_SimMode = false'
%           2.1 if using MAC_BOOK, set c_MacBook to true;
%           2.2 if using WINDOWS, set c_MacBook to false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function finalRad= hw3_team_01 (serPort)

    % constants
    global c_MacBook;
    global c_PortName;
    global c_TestingOn;

    global c_SimMode;
    global c_LoopInteval;
    global c_SlowFwdVel;
    global c_VerySlowFwdVel;
    global c_BackOffVel;
    global c_BackOffDist; % meters %

    global c_LeftTurnAngle;
    global c_RightTurnAngle;
    global c_CenterTurnAngle;
    global c_TurnSpeed;
    global c_TurnRadius;

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;

    init_global ();
    init_plotting ();

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

    % hit the wall and stop - we disable this for HW2
    %waitBump (serPort);
    %BeepRoomba (serPort);
    %SetFwdVelAngVelCreate (serPort, 0.0, 0.0);

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

            % reset moving stats
            update_moving_stats (serPort);
            if (update_current_map (2) == true)
                break;
            end
            
            deg = 0;
            
            % Turn away from obstacle: always counter clock-wise (leverage the wall sensor)
            if ((bRight && bLeft) || bCenter)
                display ('bumpReact: center')
                
                if (rand > 0.5)
                    deg = randi([90 165],1,1);
                    
                else
                    deg = randi([195 270],1,1);
                    
                    % optimization: faster turns
                    if (deg > 180)
                        deg = (deg - 180) * (-1);
                    end
                end
                
                SetLEDsRoomba (serPort, 3, 50, 50);
            elseif bRight
                display ('bumpReact: right')
                deg = randi([30 175],1,1);
                SetLEDsRoomba (serPort, 2, 0, 50);
            elseif bLeft
                display ('bumpReact: left')
                deg = randi([30 175],1,1) * (-1);
                SetLEDsRoomba (serPort, 1, 100, 50);
            end
            
            deg
            
            % back off for a little bit
            travelDist (serPort, c_BackOffVel, c_BackOffDist);
            turnAngle (serPort, c_TurnSpeed, deg);
            
            update_moving_stats (serPort);
            
            % turning is done, let the robot go straight.
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
        else
            wallSensor = WallSensorReadRoomba (serPort);
            
            if (wallSensor)
                if (update_current_map (2) == true)
                    break;
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

function isDone= circumvention (serPort)
end

function init_plotting ()    
    global figHandle;
    figHandle = figure;     
end

% This is used to fine tuning the Y location, as close as possible
function regression_state (serPort)
    
    global g_total_x_dist;
    global g_total_y_dist;
    global g_goal_dist;
    global g_total_angle;
    
    global c_VerySlowFwdVel;
    global c_TurnSpeed;
    
    % re-orient to zero
    delta_x = g_goal_dist - g_total_x_dist;
    delta_y = g_total_y_dist;
    
    checkMovingStats ();
    
    display (sprintf ('regression_state: x = %f, delta_x = %f', g_total_x_dist, delta_x));
    turnAngle (serPort, c_TurnSpeed, (-1.0) * g_total_angle);
    update_moving_stats (serPort);
    
    travelDist (serPort, c_VerySlowFwdVel, delta_x);
    update_moving_stats (serPort);
    
    checkMovingStats ();
    
    % re-orient to 90
    display (sprintf ('regression_state: y = %f, delta_y = %f', g_total_y_dist, delta_y));
    if (g_total_y_dist > 0)
        turnAngle (serPort, c_TurnSpeed, -90.0);
    else
        turnAngle (serPort, c_TurnSpeed, 90.0);
    end
    
    update_moving_stats (serPort);
    
    travelDist (serPort, c_VerySlowFwdVel, delta_y);
    update_moving_stats (serPort);
    
    checkMovingStats ();
    
    display ('leave - regression_state');
    
end

% a new state: trying to find the wall again.
function find_wall (serPort)
    
    global c_SlowFwdVel;
    global c_TurnRadius;
    global c_LoopInteval;
    global c_TurnSpeed;
    
    display ('enter - find_wall');
                    
    % The new algo is not good for the new obstacles.
    %SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
    %travelDist (serPort, c_SlowFwdVel, 0.05);
    %turnAngle (serPort, c_TurnSpeed, 5);
    update_moving_stats (serPort);
    
    % start to turn
    display ('start to turn');
    SetFwdVelRadiusRoomba (serPort, c_TurnSpeed, c_TurnRadius);
    
    while (true)
    
        update_moving_stats (serPort);
        
        % check if mission completed
        if (checkMovingStats ())
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            display ('leaving - find_wall: end point found');
            return;
        end
        
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
            display ('leaving - find_wall - bump sensor');
            return;
        else 
            if (wallSensor)
                % wall sensor activated: go straight
                display ('leaving - find_wall - wall sensor');
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
    global c_grid_size;

    global g_goal_dist;
    global g_total_dist;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_contact_x_dist;
    global g_contact_y_dist;
    global g_map_matrix;
    global g_last_unexplored_time;
    
    rng (0,'twister');

    % test-related constants
    c_SimMode           = true;
    c_MacBook           = true;
    g_goal_dist         = 4.0;  % This is golden!
    
    % environment-related constants
    c_TurnRadius        = -0.20;
    
    % declare the map (50 x 50 array)
    g_map_matrix        = ones (50);

    if c_SimMode
        c_SlowFwdVel    = 0.5;
        c_TurnSpeed     = 0.25;
        c_LoopInteval   = 0.01;
        c_VerySlowFwdVel = 0.1;
    else
        if c_MacBook
            % machine dependent params
            c_PortName      = 'ElementSerial-ElementSe';
            c_TestingOn     = true;
            c_SlowFwdVel    = 0.03;
            c_TurnSpeed     = 0.025;
            c_LoopInteval   = 0.01;
            c_VerySlowFwdVel = 0.025;
        else
            % machine dependent params
            c_PortName      = 4;
            c_TestingOn     = true;
            c_SlowFwdVel    = 0.05;
            c_TurnSpeed     = 0.05;
            c_LoopInteval   = 0.1;
            c_VerySlowFwdVel = 0.025;
        end
    end

    g_last_unexplored_time = cputime;

    c_BackOffVel        = 0.025;
    c_BackOffDist       = -0.01;

    c_LeftTurnAngle     = 60;
    c_RightTurnAngle    = 15;
    c_CenterTurnAngle   = 45;
    c_grid_size         = 0.3; % meters %

    c_MaxToleranceRadius = 0.15; % meters %

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

function isTrue= closer_to_the_goal ()

    global g_total_x_dist;
    global g_total_y_dist;
    global g_contact_x_dist;
    global g_contact_y_dist;
    global g_goal_dist;
    
    dist_new = abs (g_total_x_dist - g_goal_dist);
    dist_old = abs (g_contact_x_dist - g_goal_dist);
    display (sprintf ('closer_to_the_goal: new = %f, old = %f', dist_new, dist_old));
    
    if (dist_new < dist_old)
        display ('closer to goal');
        isTrue = true;
    else
        display (sprintf ('not closer to goal'));
        isTrue = false;
    end
end

% check if the bump point is reached 'again'
function hit= hit_bumping_pt ()

    global g_contact_x_dist;
    global g_contact_y_dist;
    global g_total_x_dist_after_bump;
    global g_total_y_dist_after_bump;
    
    if (dist_after_bumping () > 0.2)
    
        d = sqrt ((g_total_x_dist_after_bump - g_contact_x_dist)^2 + (g_total_y_dist_after_bump - g_contact_y_dist)^2); 
    
        if (d < 0.10)
            hit = true;
            return;
        end
    end
    
    hit = false;
    return;
end

% return the 'absolute' distance after bumping.
function dist= dist_after_bumping ()

    global g_abs_dsit_after_bump;
    dist = g_abs_dsit_after_bump;
end

function reset_bumping_moving_status ()

    global g_contact_x_dist;
    global g_contact_y_dist;
    global g_abs_dsit_after_bump;
    global g_total_x_dist_after_bump;
    global g_total_y_dist_after_bump;
    global g_total_x_dist;
    global g_total_y_dist;
    
    g_abs_dsit_after_bump = 0;
    g_total_x_dist_after_bump = 0.0;
    g_total_y_dist_after_bump = 0.0;
    
    % remember the current bump point.
    g_contact_x_dist    = g_total_x_dist;
    g_contact_y_dist    = g_total_y_dist;
end

% Update the current map
% status:
%   0: explored 
%   1: unknown --> not possible
%   2: wall
function isDone= update_current_map (status)

    global g_total_x_dist;
    global g_total_y_dist;
    global c_grid_size;
    global g_map_matrix;
    global figHandle;
    global g_last_unexplored_time;
    global g_total_angle;
    
    isDone = false;
    
    % update current matrix info
    x_idx = round (g_total_x_dist / c_grid_size) + 25;
    y_idx = round (g_total_y_dist / c_grid_size) + 25;
    
    tmp = g_map_matrix (y_idx, x_idx);
    
    if (tmp == 2)
        return;
    end
    
    if (tmp == status)
        return;
    end
    
    % calculate stop time.
    if (status == 0 && tmp == 1)
        diff = cputime - g_last_unexplored_time;
        
        if (diff > 60)
            isDone = true;
        else
            g_last_unexplored_time = cputime;
        end
    end
    
    g_last_unexplored_time
    g_map_matrix (y_idx, x_idx) = status;
    
    figure (figHandle);
    [r,c] = size (g_map_matrix);                  
    imagesc ((1:c) + 0.5, (1:r) + 0.5, g_map_matrix);           
    colormap (gray);                             
    axis equal;                                  
    set(gca,'XTick',1:(c+1),'YTick',1:(r+1),...  
            'XLim', [1 c+1],'YLim',[1 r+1],...
            'GridLineStyle','-','XGrid','on','YGrid','on');
end

% This is buggy -
function isDone= update_moving_stats (serPort)

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;
    global g_total_x_dist_after_bump;
    global g_total_y_dist_after_bump;
    global g_abs_dsit_after_bump;
    global c_grid_size;
    global g_map_matrix;

    dist = DistanceSensorRoomba (serPort);
    angle = AngleSensorRoomba (serPort);
    
    if (isnan (dist) | isnan (angle))
        display ('!!! Bad Comm !!!');
        return;
    end
    
    g_total_dist  = g_total_dist + dist;
    g_total_angle = g_total_angle + angle;
    
    temp_x = dist * cos (g_total_angle);
    temp_y = dist * sin (g_total_angle);

    g_total_x_dist = g_total_x_dist + temp_x;
    g_total_y_dist = g_total_y_dist + temp_y;

    g_total_x_dist_after_bump = g_total_x_dist_after_bump + temp_x;
    g_total_y_dist_after_bump = g_total_y_dist_after_bump + temp_y;

    g_abs_dsit_after_bump = g_abs_dsit_after_bump + abs(dist);
    
    isDone = update_current_map (0);
end

% TODO: check if there is any 'connected unexplored' block
function isDone= checkMovingStats ()
    isDone = false;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @lfred: a small trick to stop the robot right after the wall is hit
function waitBump (serPort)

    global c_SimMode;

    if (c_SimMode)
        while (true)
            [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);
            
            if (bRight | bLeft | bCenter)
                return;
            end
                
            pause (0.01);
        end
    else
        try
            % marked for test
            %set (serPort,'timeout',.01);

            %Flush buffer
            N = serPort.BytesAvailable();

            while (N ~= 0)
                fread (serPort, N);
                N = serPort.BytesAvailable ();
            end
            
            %% Get (142) Wall Reading(8) data fields
            global td;
            fwrite (serPort, [158 5]);
            pause (td);
        catch
            disp('WARNING:  waitBump function failed.')
        end
    end
    
end
