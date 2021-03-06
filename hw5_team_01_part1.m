%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 5 part 1 (color tracker)
%
% Team number: 1
% Team leader:  Jen-Chieh Huang (jh3478)
% Team members: Sze wun wong (sw2955)
%               Duo Chen (dc3026)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function hw5_team_01_part_1 ()
    serPort = RoombaInit (4);
    global roi_min;
    global roi_tolerance;
    global roi_not_found;

    init_globals ();

    while (true)
        
        % get a frame each loop
        [com, roi] = camera ();
        
        if (roi < 0)
            display('stuck in roi < 0');
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            continue;
        end
        
        % if I can not find the object, turning to locate
        if (roi < roi_not_found)
            roi
            display('stuck in roi min');
            find_object (serPort);
        else
            display('aiming the object!!!');
            aiming_the_object (serPort, com);
            display('determining the distance!!!');
            determin_distance (serPort, roi);
        end
    end
end

% The function we used to retrieve the image
function [center_of_mass, region_of_interest] = camera ()

    % @lfred: a trick here - reduce the frequency of taking pictures.
    while (true)
        try
            display ('*** say cheese ***');
            image = imread ('http://192.168.0.100/img/snapshot.cgi?');
            pause (0.2);
            break;
        catch
            continue;
        end
    end
    
    % do the image processing job
    [bw, maskedRGBImage] = createMask (image);
    se = strel ('square', 3);
    ed1 = imerode (bw, se);
    
    se2 = strel ('square', 12);
    ed = imdilate (bw, se2);
    ap = regionprops(ed, 'area');
    aarray = cat (1, ap.Area);
    cp = regionprops(ed, 'centroid');
    carray = cat (1, cp.Centroid);
    
    [max1, index] = max (aarray);
    
    if (length(index) ~= 0)
        % output the carray
        x = carray (index, 1)
        y = carray (index, 2)

        imshow (ed);
        hold on;
        plot (x, y, '*');
        hold off;

        center_of_mass = [x, y];
        region_of_interest = max1;
    else
        center_of_mass = [-1, -1];
        region_of_interest = -1;
    end
end

function determin_distance (serPort, roi)

    global roi_tolerance;
    global roi_min;
    global roi_not_found;
    
    %while (true)
        
        if (abs (roi - roi_min) < roi_tolerance)
            display ('okay - this is good enough');
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0)
            return;
        end
        
        if (roi > roi_min)
            % go back
            SetFwdVelAngVelCreate (serPort, -0.1, 0.0)
        else
            % go forward
            SetFwdVelAngVelCreate (serPort, 0.1, 0.0)
        end
        
        % get a frame each loop
    %    [com, roi] = camera ();
        
    %    if (roi < 0)
    %        return;
    %    end
    %end
    
    % stop the robot
    %SetFwdVelAngVelCreate (serPort, 0.0, 0.0)
end

function aiming_the_object (serPort, com)
    
    display ('aiming_the_object');
    turn = 16
    
    if (com(1) < 160)
        side = 1;
    else
        side = 0;
    end

    % aiming the target
    while (abs(com(1)-160) > 20)
    
        display ('aiming_the_object: 1' );
        
        % end condition to avoid continuously turn
        if (turn < 1)
            break;
        end
        
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        
        if (com(1) < 160)

            if (side == 0)
                turn = turn/2;
                side = 1;
            end
            turn_to_target (serPort, turn);
            display('turned right!');
        else

            if (side == 1)
                turn = turn/2;
                side = 0;
            end

            turn_to_target (serPort, (-1) * turn);
            display('turned left!');
        end

        [com, roi] = camera();
        
        if (roi < 0)
            return;
        end
    end
    
    stats = true;
end

% TO_CHECK
function stats = find_object (serPort)

    % globals
    global roi_min;
    SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
    % locals
    acc_angle = 0;
    unit_turn = 30;
    
    % find the object
    while (1)
    
        % get current image
        [com, roi] = camera ();
        
        if (roi < 0)
            stats = false;
        end
        
        if (acc_angle >= 360)
            display ('I can not find the target - sorry.');
            stats = false;
            return;
        end
        
        if (roi > roi_min) 
            turn_to_target (serPort, unit_turn);
            acc_angle = acc_angle + unit_turn;
        else
            break;
        end
    end
    
    stats = true;
end

% turning the robot to the target point
function turn_to_target (serPort, angle)
    
    thisAngle = angle;
    SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
    AngleSensorRoomba (serPort);
    read_angle = 0;
    
    while (true)
        turnAngle (serPort, 0.025, thisAngle);
        read_angle = read_angle + AngleSensorRoomba (serPort);
        
        if (abs (read_angle - deg2rad(angle)) < deg2rad(1))
            break;
        else
            display ('Force tuning');
            thisAngle = rad2deg(deg2rad(angle) - read_angle);
        end
    end
end

function rad=  deg2rad (deg)
    rad = deg * pi / 180;
end

function deg= rad2deg (rad)
    deg = rad * 180 / pi;
end

function center_of_mass = COM(BW)
    binaryImage = true(size(BW));
    labeledImage = bwlabel(binaryImage);
    measurements = regionprops(labeledImage, BW, 'WeightedCentroid');
    center_of_mass = measurements.WeightedCentroid;
end

function init_globals ()
    global roi_min
    global roi_tolerance
    global roi_not_found
        roi_not_found = 25*25;
        roi_min = 100 * 100;
        roi_tolerance = roi_min * .25;
end
