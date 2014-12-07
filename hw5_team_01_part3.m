%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 5 part 3 (corridor tracking)
%
% Team number: 1
% Team leader:  Jen-Chieh Huang (jh3478)
% Team members: Sze wun wong (sw2955)
%               Duo Chen (dc3026)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function hw5_team_01_part_3 (serPort)
    
    % assumptions: 
    %   according to the assignment description, 
    %   we can assume the we are in the middle of the hall way.
    
    % tracking features - the light and the green tube
    %   - The light - track the light to prevent us from going stray
    %   - The tube - allow us to know when to turn (X) --> the color is not working.
    
    % TODO: homing test
    % step 0. process the image and try to follow the light
    while (true)
        
        img = take_picture ();
        
        roi = find_light (img);
        [x, y, a] = find_largest_roi (roi);
        
        if a != -1
            % turn to (x, y)
            turn_to_point (x);
        else
            % left turn 
            turn_to_target (serPort, 90);
            
            img_left = take_picture ();
            roi_left = find_light (img_left);
            [xl, yl, al] = find_largest_roi (roi_left);
            
            if al != -1
                continue;
            end
            
            % right turn 
            turn_to_target (serPort, 180);
                
            img_right = take_picture ();
            roi_right = find_light (img_left);
            [xr, yr, ar] = find_largest_roi (roi_right);
            
            if ar == -1
                display ('hell - there is no light ? - exit');
                return;
            end
        end
        
        % step 1. see if we are home.
    end
end

% TODO: aiming the light
function turn_to_point (x)

end

% input: roi array, returned by the find_light
% if roi is not found, [-1, -1, -1] is returned.
function [x, y, a]= find_largest_roi (roi_array)

    area_array = cat (1, s.Area);
    [max, index] = max (area_array);
    
    if max > 30*30 or max < 5*5:
        x = -1;
        y = -1;
        a = -1;
    else
        x = roi_array (index).Centroid (1);
        y = roi_array (index).Centroid (2);
        a = max;
    end
end

% find all light sources at this moment
function prop= find_light (img)

    % do gray scale
    I = rgb2gray (img);

    % mask the bottom 1/3
    centerIndex = round (size(I,1) / 3 * 2);
    I(centerIndex:end,:) = cast (0, class(I));

    % do threasholding for lights
    bw = im2bw (I, 0.97);
    se = strel ('square', 7);
    ed = imerode (bw, se);
    s  = regionprops (ed, 'centroid');
    centroids = cat(1, s.Centroid);

    %imshow (ed)
    %hold on
    %plot(centroids(:,1), centroids(:,2), 'b*')
    %hold off
    
    prop = s;
end

function stats = correctPath (serPort)

    com = camera();
    acc_angle = 0;
    
    % find the object
    while (1)
        if (acc_angle >= 360)
            display ('I can not find the image - fuck you.');
            stats = false;
            return;
        end
        
        if (com == [-1, -1]) 
            turn_to_target (serPort, 30);
            acc_angle = acc_angle + 30;
            com = camera ();
        else
            break;
        end
    end
    
    turn = 16
    
    if (com(1) < 160)
        side = 1;
    else
        side = 0;
    end

    % aiming the target
    while (abs(com(1)-160) > 20)
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        if(com(1) < 160)

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

        com = camera();
    end
    
    stats = true;
end

function move_to_next_pt(serPort,pts)
    current_ang = 0;
    x = pts(:,1);
    y = pts(:,2);

    for i = 1:length(x)-1
        theta = atan2d((x(i+1) - x(i)),(y(i+1) - y(i)));
        turn_ang = theta - current_ang;
        current_ang = current_ang + turn_ang;
        turn_ang = -turn_ang
        dist = sqrt((y(i+1) - y(i))^2 + (x(i+1) - x(i))^2);
        turn_to_target (serPort, turn_ang);
        travelDist (serPort, .22, dist);
    end
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
    
    function deg= rad2deg (rad)
        deg = rad * 180 / pi;
    end
end

function rad=  deg2rad (deg)
    rad = deg * pi / 180;
end

function deg= rad2deg (rad)
    deg = rad * 180 / pi;
end

function image= take_picture ()
    image = imread ('http://192.168.0.101/img/snapshot.cgi?');
end

function center_of_mass = camera ()

    image = imread('http://192.168.0.101/img/snapshot.cgi?');
    [BW, maskedRGBImage] = createMask(image);
    [x, y] = size(BW)
    imshow(BW);
    
    if (nnz(BW) > 15*15)
        center_of_mass = COM(BW);
    else
        center_of_mass = [-1, -1];
    end
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

function center_of_mass = COM (BW)
    binaryImage = true(size(BW));
    labeledImage = bwlabel(binaryImage);
    measurements = regionprops(labeledImage, BW, 'WeightedCentroid');
    center_of_mass = measurements.WeightedCentroid;
end
