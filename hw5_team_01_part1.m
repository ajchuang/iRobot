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
function hw5_team_01_part_1 (serPort)

    global roi_min;
    global roi_tolerance;

    init_globals ();

    while (true)
    
        % get a frame each loop
        [com, roi] = camera ();
        
        % if I can not find the object, turning to locate
        if (roi < roi_min)
            find_object (serPort);
        else
            aiming_the_object (serPort, com);
            determin_distance (serPort, roi);
        end
        % SetFwdVelAngVelCreate (serPort, 0.2, 0.0);
    end
end

% The function we used to retrieve the image
function [center_of_mass, region_of_interest] = camera ()

    image = imread ('http://192.168.0.101/img/snapshot.cgi?');
    [BW, maskedRGBImage] = createMask (image);
    [x, y] = size (BW);
    % imshow (BW);

    [center_of_mass, region_of_interest] = [COM (BW), nnz (BW)];
end

% TODO
function determin_distance (serPort, roi)

    global roi_tolerance;
    global roi_min;
    
    while (true)
        
        if (abs (roi - roi_min) < roi_tolerance)
            display ('okay - this is good enough');
            break;
        end
        
        if (roi > roi_min)
            % go back
            SetFwdVelAngVel (serPort, -0.5, 0.0)
        else
            % go forward
            SetFwdVelAngVel (serPort, 0.5, 0.0)
        end
        
        % get a frame each loop
        [com, roi] = camera ();
    end
    
    % stop the robot
    SetFwdVelAngVel (serPort, 0.0, 0.0)
end

function aiming_the_object (serPort, com)
    
    turn = 16
    
    if (com(1) < 160)
        side = 1;
    else
        side = 0;
    end

    % aiming the target
    while (abs(com(1)-160) > 20)
    
        % end condition to avoid continuously turn
        if (turn < 1)
            break;
            
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        
        if (com(1) < 160)

            if (side == 0)
                turn = turn/2;
                side = 1
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
    end
    
    stats = true;
end

% TO_CHECK
function stats= find_object (serPort)

    % globals
    global roi_min;
    
    % locals
    acc_angle = 0;
    unit_turn = 30;
    
    % find the object
    while (1)
    
        % get current image
        [com, roi] = camera ();
        
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

function [BW,maskedRGBImage] = createMask(RGB)
%createMask Threshold RGB image using auto-generated code from colorThresholder app.
% [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using 
% auto-generated code from the colorThresholder App. The colorspace and
% minimum/maximum values for each channel of the colorspace were set in the 
% App and result in a binary mask BW and a composite image maskedRGBImage,
% which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 03-Jun-2014
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0;
channel1Max = 1;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.05;
channel2Max = .5;

% Define thresholds for channel 3 based on histogram settings
channel3Min = .9;
channel3Max = 1;

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Invert mask
%BW = ~BW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end

function center_of_mass = COM(BW)
    binaryImage = true(size(BW));
    labeledImage = bwlabel(binaryImage);
    measurements = regionprops(labeledImage, BW, 'WeightedCentroid');
    center_of_mass = measurements.WeightedCentroid;
end

function init_globals ()
    global roi_min = 15x15;
    global roi_tolerance = 100;
end
