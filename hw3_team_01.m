%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 3
%
% Team number: 1
% Team leader:  Jen-Chieh Huang (jh3478)
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

    init_world ();
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
    
    % do bug to find the wall
    reset_bug_params ([4.0, 0.0]);
    turn_to_target (serPort, [4.0, 0.0]);
    bug_algo (serPort, [25, 49]);
    
    % set up walls
    wall_annotation ();
    
    while (true)
    
        % pick a 'connected' unexplored blk.
        nextblk = any_unexplored_blks ();
        
        % log
        nextblk
        
        if (nextblk == [-1, -1])
            display ('no blocks to explore - complete');
            break;
        end
        
        % returns when the designated block becomes explored or obstacle.
        go_to_block (serPort, nextblk);
        pause (c_LoopInteval);
    end
    
    % do other jobs
    
    % clean up
    if c_SimMode == false
        fclose (serPort);
        delete (serPort);
        clear  serPort;
    end

    % Specify output parameter
    finalRad = g_total_angle;
end

% turning the robot to the target point
function turn_to_target (serPort, M)

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global c_TurnSpeed;
    
    theta = atan2 (M(2) - g_total_y_dist, M(1) - g_total_x_dist);
    angle = rad2deg (((-1) * g_total_angle + theta));
    
    angle = mod (round(angle), 360);
    
    if (angle > 180)
        angle = (180 - (angle - 180)) * (-1);
    end 
    
    
    %log
    angle
    
    thisAngle = angle;
    SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
    
    while (true)
        before = g_total_angle;
        turnAngle (serPort, 0.15, thisAngle);
        update_moving_stats (serPort);
        after = g_total_angle;
        
        thisAngle = thisAngle - rad2deg (after - before);
        thisAngle = mod (round(thisAngle), 360);
        
        if (thisAngle > 180)
            thisAngle = (180 - (thisAngle - 180)) * (-1);
        end
        
        if (abs (thisAngle) < 5)
            break;
        else
            display (sprintf ('insufficient angle: %f', thisAngle));
            continue;
        end
    end    
end

function go_to_target (serPort, M)

    global g_total_x_dist;
    global g_total_y_dist;
    global c_SlowFwdVel;
    global c_LoopInteval;
    
    global c_blk_explored;
    global c_blk_unexplored;
    global c_blk_wall;
    global c_blk_bound;
    global c_MaxToleranceRadius;
    
    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
    
    while (true)
    
        % update current position
        update_moving_stats (serPort);
        
        if (point_distance (M, [g_total_x_dist, g_total_y_dist]) < c_MaxToleranceRadius)
        
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            travelDist (serPort, c_SlowFwdVel, 0.15);
            update_moving_stats (serPort);
        
            update_current_map (c_blk_explored);
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            
            display ('Arrived the designated point - return');
            return;
        end
        
        [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);

        if (isnan (bRight) || isnan (bCenter) || isnan (bLeft))
            display ('So bad - I dont know whats that');
            continue;
        else
            bumped = bRight | bCenter | bLeft;
        end
        
        if (bumped)
            display ('Bumpped - return');
            update_current_map (c_blk_wall);
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            return;
        end 
        
        pause (c_LoopInteval);
    end
end

function go_to_block (serPort, blk)
    
    % translate the coordinates
    coord_mat = translate_blk_to_coord (blk);
    reset_bug_params (coord_mat);
    
    % turn to the target
    turn_to_target (serPort, coord_mat);
    
    % go to destination
    bug_algo (serPort, blk);
end

function unexplored_blk= any_unexplored_blks ()

    global g_map_matrix;
    
    for i = 1:50
        for j = 1:50
            if (g_map_matrix(i, j) == 0)
                
                if (i > 1 && g_map_matrix (i - 1, j) == 1)
                    unexplored_blk = [i - 1, j]; 
                    return;
                end 
                
                if (i < 50 && g_map_matrix (i + 1, j) == 1)
                    unexplored_blk = [i + 1, j]; 
                    return;
                end
                
                if (j < 50 && g_map_matrix (i, j + 1) == 1)
                    unexplored_blk = [i, j + 1]; 
                    return;
                end 
                
                if (j > 1 && g_map_matrix (i, j - 1) == 1)
                    unexplored_blk = [i, j - 1]; 
                    return;
                end
            end
        end
    end
  
    unexplored_blk = [-1, -1];
end

function wall_annot_rec (x, y)

    global g_map_matrix;
    
    global c_blk_bound;
    global c_blk_wall;

    tmp = g_map_matrix (x, y);
    
    if (tmp == c_blk_wall)
        g_map_matrix (x, y) = c_blk_bound;
    else
        return;
    end
    
    % do recusion
    wall_annot_rec (x, y + 1);
    wall_annot_rec (x, y - 1);
    wall_annot_rec (x + 1, y);
    wall_annot_rec (x - 1, y);
    
    return;
end

% status:
%   0: explored 
%   1: unknown --> not possible
%   2: wall
%   3: boundary
function wall_annotation ()

    global g_total_x_dist;
    global g_total_y_dist;
    global c_grid_size;
    global c_map_dim;
    global c_x_origin_grid;
    global c_y_origin_grid;
    
    global c_blk_explored;
    global c_blk_unexplored;
    global c_blk_wall;
    global c_blk_bound;
    
    blk = translate_coord_to_blk ([g_total_x_dist, g_total_y_dist]);
    wall_annot_rec (blk (1), blk (2));
    
    % repaint the whole map
    do_plotting ();
end

% to impl
function reset_bug_params (dst)
    global g_goal_point;
    global g_found_box;
    global g_contact_x_dist;
    global g_contact_y_dist;

    g_goal_dist         = dst; 
    g_found_box         = false;
    g_contact_x_dist    = 0.0;
    g_contact_y_dist    = 0.0;
    
    reset_bumping_moving_status ();
end

function bug_algo (serPort, dst_blk)

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
    global c_TurnRadius;
    global c_blk_wall;
    global c_blk_explored;

    global g_found_box;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_map_matrix;

    % loop variables
    wallSensor  = false;
    bumped      = false;
    p_bRight    = false;
    p_bCenter   = false;
    p_bLeft     = false;
    p_ang       = 0;

    % Start robot moving: go straight
    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);

    % Enter main loop: control cycle is about 2.3 (sec)
    while (true)

        % turn off the Roomba lights
        SetLEDsRoomba (serPort, 0, 0, 1);

        % step 0: update last readings
        update_moving_stats (serPort);

        % check if mission completed
        if (checkMovingStats () || g_map_matrix (dst_blk(1), dst_blk(2)) ~= 1)
            display ('Found the end point - Stop!')
            break;
        end
        
        % check if we should leave the obstacle.
        if (checkExitObstacle (serPort) == true)
            display ('!!! unable to reach the goal - sorry !!!')
            return;
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
            
            % set current location as 'obstacle'
            update_current_map (c_blk_wall);

            display ('bump into the wall');

            if (g_found_box == false)
                % reset moving stats
                update_moving_stats (serPort);
                
                % set when the box is found
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
                    
                    % also update the current location as wall
                    update_current_map (c_blk_wall);

                    % a minor optimization using Wall Sensor
                    SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0.0);
                else
                    find_wall (serPort);
                end
            else
                % also update the current location as traversed
                update_current_map (c_blk_explored);
            end
        end

        % Briefly pause to avoid continuous loop iteration
        pause (c_LoopInteval);
    end
    
    update_moving_stats (serPort);
    SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
end

% Update the current map
function isDone= update_current_map (status)

    global g_total_x_dist;
    global g_total_y_dist;
    global c_grid_size;
    global g_map_matrix;
    global figHandle;
    global g_last_unexplored_time;
    global g_total_angle;
    
    global c_blk_explored;
    global c_blk_unexplored;
    global c_blk_wall;
    global c_blk_bound;
    
    isDone = false;
    cordM = translate_coord_to_blk ([g_total_x_dist, g_total_y_dist]);
    
    x_idx = cordM(1);
    y_idx = cordM(2);
        
    tmp = g_map_matrix (x_idx, y_idx);
    
    if (tmp == c_blk_wall || tmp == c_blk_bound)
        return;
    end
    
    if (tmp == status)
        return;
    end
    
    % calculate stop time.
    if (status == c_blk_explored && tmp == c_blk_unexplored)
        diff = cputime - g_last_unexplored_time;
        
        if (diff > 60)
            isDone = true;
        else
            g_last_unexplored_time = cputime;
        end
    end
    
    g_map_matrix (x_idx, y_idx) = status;
    
    do_plotting ();
end

% TODO
% there is a hard-coded constant
% Corner case: need to be handled.
function neverEnd= checkExitObstacle (serPort)

    global g_found_box;
    global c_MaxToleranceRadius;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_contact_x_dist;
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
        
        if (hit_bumping_pt ())
            neverEnd = true;
            return;
        end
        
        if (dist_after_bumping () > 0.20 && closer_to_the_goal ())
            display ('Leaving the box');
            
            % reset bumping memories
            g_found_box = false;
            c_SlowFwdVel = c_SlowFwdVel * 1.3;
            reset_bumping_moving_status ();
            
            % stop and turn
            if (g_total_x_dist > g_goal_dist)
                turnDeg = g_total_angle * 180.0 / pi;
            else
                turnDeg = g_total_angle * (-1) * 180.0 / pi;
            end
            
            SetFwdVelAngVelCreate (serPort, 0.0, 0);
            turnAngle (serPort, c_TurnSpeed, turnDeg);
            update_moving_stats (serPort);
            
            % keep going
            SetFwdVelAngVelCreate (serPort, c_SlowFwdVel, 0);
            return;
        end
    end 
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
            update_moving_stats (serPort);
            display ('leaving - find_wall: end point found');
            return;
        end
        
        % check if we should leave the obstacle.
        if (checkExitObstacle (serPort) == true)
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            update_moving_stats (serPort);
            display ('leaving - find_wall: unable to reach the goal');
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

    % current pos
    global g_total_x_dist;
    global g_total_y_dist;
    
    % impact pos
    global g_contact_x_dist;
    global g_contact_y_dist;
    
    % destination
    global g_goal_dist;
    
    global c_MaxToleranceRadius;
    
    b_is_mline = false;
    s = Inf;
    dist = Inf;
    
    % log
    tmp = g_goal_dist - [g_contact_x_dist, g_contact_y_dist];
    
    if (tmp (1) ~= 0 && tmp (2) ~= 0)
        s = tmp (2) / tmp (1);
    end
    
    if (s ~= Inf && s ~= 0)
        c = (-1) * s * g_goal_dist (1) + g_goal_dist (2);
        b = 1 / s * g_total_x_dist + g_total_y_dist;
        
        intersec_x = s / (1 + s .^ 2) * (b - c);
        intersec_y = (s .^ 2) / ( 1 + s .^ 2) * (b - c) + c;
        
        %log 
        s
        b
        c
        intersec_x
        intersec_y
        g_goal_dist
        g_total_x_dist        
        g_total_y_dist
        
        dist = point_distance ([g_total_x_dist, g_total_y_dist], [intersec_x, intersec_y]);
        
        %log
        dist
    else
        if (tmp (2)==0)
            dist = abs (g_total_y_dist - g_goal_dist (2));
        else
            % s == inf
            dist = abs (g_total_x_dist - g_goal_dist (1));
        end
    end
    
    display (sprintf ('current dist %f', dist));
    
    if (dist < 0.1)
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
    
    dist_old = point_distance ([g_contact_x_dist, g_contact_y_dist], g_goal_dist);
    dist_new = point_distance ([g_total_x_dist, g_total_y_dist], g_goal_dist);

    display (sprintf ('closer_to_the_goal: new = %f, old = %f', dist_new, dist_old));
    
    % trick - to avoid failing to exit
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
    global g_total_x_dist;
    global g_total_y_dist;
    
    % set hit as false
    hit = false;
    
    if (dist_after_bumping () > 0.3)
    
        d = sqrt ((g_total_x_dist - g_contact_x_dist) .^ 2 + ...
                  (g_total_y_dist - g_contact_y_dist) .^ 2);
        
        distance_to_hit_point = d
        
        if (d <= 0.15)
            hit = true;
            return;
        end
    end
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

% This is buggy -
function update_moving_stats (serPort)

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global g_total_dist;
    global g_total_x_dist_after_bump;
    global g_total_y_dist_after_bump;
    global g_abs_dsit_after_bump;

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
end

function isDone= checkMovingStats ()

    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    global c_MaxToleranceRadius;
    global g_goal_dist;

    radius = sqrt ((g_total_x_dist - g_goal_dist) .^ 2 + g_total_y_dist .^ 2);

    display (sprintf ('current radius = %f', radius));
    display (sprintf ('current g_total_x_dist = %f, g_total_y_dist = %f', g_total_x_dist, g_total_y_dist));

    if (radius < c_MaxToleranceRadius)
        isDone = true;
    else
        isDone = false;
    end
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               UI function                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function init_plotting ()
    global figHandle;
    figHandle = figure; 
end

function do_plotting ()

    global figHandle;
    global g_map_matrix;

    figure (figHandle);
    [r,c] = size (g_map_matrix);                  
    
    imagesc ((1:c) + 0.5, (1:r) + 0.5, (g_map_matrix / 4));
    colormap (gray);
    axis equal;
    
    set (gca,   'XTick',    1:(c+1), ...
                'YTick',    1:(r+1), ...  
                'XLim',     [1 c+1], ...
                'YLim',     [1 r+1], ...
                'GridLineStyle',    '-', ...
                'XGrid',    'on', ...
                'YGrid',    'on');

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               UTILITIES                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input     blk:    [row,   col ]
% output    coord:  [x,     y   ]
function coord= translate_blk_to_coord (blk)

    global c_map_dim;
    global c_x_origin_grid;
    global c_y_origin_grid;
    global c_grid_size;     % grid edge length in meters
    global c_map_dim;       % the num of blks of an edge

    xc = (blk(2) - c_x_origin_grid) * c_grid_size;
    yc = (c_map_dim - blk(1) - c_y_origin_grid) * c_grid_size;
    
    % row --> y coord
    % col --> x coord
    coord = [xc, yc];
    
    %log
    coord
end

% output    coord:  [x,     y   ]
% input     blk:    [row,   col ]
function blk= translate_coord_to_blk (coord)

    global c_x_origin_grid; 
    global c_y_origin_grid;
    global c_grid_size;     % grid edge length in meters
    global c_map_dim;       % the num of blks of an edge
    
    coord = round (coord / c_grid_size);

    row = c_y_origin_grid - coord(2);
    col = c_x_origin_grid + coord(1);
    
    % y and x -- this is NOT typo
    blk = [row, col];
    
    %log 
    blk
end

% return the distance of 2 points
function dist= point_distance (src, dst)
    tmp = (dst - src).^2;
    dist = sqrt (tmp(1) + tmp(2));
end

function rad=  deg2rad (deg)
    rad = deg * pi / 180;
end

function deg= rad2deg (rad)
    deg = rad * 180 / pi;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               INITIALIZATION                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function init_const ()

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
    
    % for HW3
    global c_grid_size;
    global c_map_dim;   % the num of blks of an edge
    global c_x_origin_grid;
    global c_y_origin_grid;
    
    global c_blk_explored;
    global c_blk_unexplored;
    global c_blk_wall;
    global c_blk_bound;
    % end
    
    % test-related constants
    c_SimMode           = false;
    c_MacBook           = false;
    
    % environment-related constants
    c_FastFwdVel        = 0.05;
    c_TurnRadius        = -0.20;
    
    if c_SimMode
        c_SlowFwdVel    = 0.3;
        c_TurnSpeed     = 0.2;
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
            c_PortName      = 3;
            c_TestingOn     = true;
            c_SlowFwdVel    = 0.05;
            c_TurnSpeed     = 0.05;
            c_LoopInteval   = 0.1;
            c_VerySlowFwdVel = 0.025;
        end
    end

    % for map exploration
    c_grid_size         = 0.3;
    c_map_dim           = 50;
    c_x_origin_grid     = 25;
    c_y_origin_grid     = 25;
    
    c_blk_explored      = 0;
    c_blk_unexplored    = 1;
    c_blk_wall          = 2;
    c_blk_bound         = 3;
    % end
    
    c_AfterBumpFwdVel   = 0.075;

    c_BackOffVel        = 0.025;
    c_BackOffDist       = -0.01;

    c_LeftTurnAngle     = 60;
    c_RightTurnAngle    = 15;
    c_CenterTurnAngle   = 45;

    c_MaxToleranceRadius = 0.10; % meters %
end

function init_bug ()

    global g_goal_dist;
    global g_found_box;
    global g_contact_x_dist;
    global g_contact_y_dist;

    g_goal_dist         = [15.0, 0.0];
    g_found_box         = false;
    g_contact_x_dist    = 0.0;
    g_contact_y_dist    = 0.0;
end

% init all global variables
function init_global ()

    global g_map_matrix;
    global g_total_dist;
    global g_total_x_dist;
    global g_total_y_dist;
    global g_total_angle;
    
    % referred constants
    global c_map_dim;

    % declare the map (50 x 50 array)
    g_map_matrix        = ones (c_map_dim);

    % The flag is used to indicate if the obstable is seen.
    g_total_dist        = 0;
    g_total_x_dist      = 0.0;
    g_total_y_dist      = 0.0;
    g_total_angle       = 0.0;
end

function init_world ()

    % do everything
    init_const ();
    init_global ();
    init_bug ();
end
