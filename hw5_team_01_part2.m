%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 5 part 2 (door finder)
%
% Team number: 1
% Team leader:  Jen-Chieh Huang (jh3478)
% Team members: Sze wun wong (sw2955)
%               Duo Chen (dc3026)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function hw5_team_01_part_2 ()

serPort = RoombaInit (4);


tuneOrientation(serPort);
display ('tuned!');

[com, roi] = camera();

while (roi < 20*20)
    travelDist(serPort, .3, 1);
    [com, roi] = camera();
    tuneOrientation(serPort);
end
display('door found!')
if (com(1) < 160)
    % door on the left side
    door_side = 1;
else
    % door on the right side
    door_side = -1;
end


while (true)
    try_distance = 1;
    travelDist(serPort, .3, try_distance);
    total_angle_turned = findDoor(serPort, door_side);
    if (abs(total_angle_turned) > 75)
        break;
    end
    turn_to_target(serPort, (-1)*total_angle_turned);

    [com, roi] = camera();

end

display('final approach!!!');

SetFwdVelAngVelCreate (serPort, 0.1, 0.0);

while (true)
    [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);
    if (bRight | bCenter | bLeft)
        display ('Home!');
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        break;
    end
end

travelDist (serPort, 0.1, -0.05);
travelDist (serPort, 0.1, 0.06);
travelDist (serPort, 0.1, -0.03);
travelDist (serPort, 0.1, 0.04);

end

    

% while (true) 
    

%     com = camera();
%     line = determine_line(com(1), 320, 3/4);
%     findDoor (serPort, line);
%     display ('found door!!');
%     SetFwdVelAngVelCreate (serPort, 0.22, 0.0);
    
%     [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);

%     if (bRight | bCenter | bLeft)
%         display ('Home!');
%         SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
%         return;
%     end
% end



% SetFwdVelAngVelCreate (serPort, 0.2, 0.0);



function stats = correctPath (serPort)

    [com, roi] = camera ();
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
            [com, roi] = camera ();
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
        
        [com, roi] = camera();
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


function [center_of_mass, region_of_interest] = camera_old()

image = imread('http://192.168.0.100/img/snapshot.cgi?');
[BW, maskedRGBImage] = createMask(image);
[x, y] = size(BW)
imshow(BW);

if (nnz(BW) > 15*15)
    center_of_mass = COM(BW);
else
    center_of_mass = [-1, -1];
end
end

function center_of_mass = COM(BW)
binaryImage = true(size(BW));
labeledImage = bwlabel(binaryImage);
measurements = regionprops(labeledImage, BW, 'WeightedCentroid');
center_of_mass = measurements.WeightedCentroid;
end

function total_angle_turned = findDoor(serPort, door_side)
    total_angle_turned = 0;
    [com, roi]  = camera();
    turn = 16;

    % check whether the door is seen. If not, find it

    max_roi = 0;
    max_angle = 0;

    for i = 1:4
        x = 30 * i;
        turn_to_target(serPort, 30 * door_side);
        [com, roi] = camera();
        if (roi > max_roi)
            max_angle = x;
        end
    end

    turn_to_target(serPort, (-1) * door_side * (120 - max_angle));
    total_angle_turned = door_side * max_angle;

    if (com(1) < 160)
        side = 1;
    else
        side = 0;
    end

    while (abs(com(1) - 160) > 40)
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        if (turn == 1)
            break;
        end
        if(com(1) < 160)
            if (side == 0)
                turn = turn/2;
                side = 1;
            end
            turn_to_target (serPort, turn);
            total_angle_turned = total_angle_turned + turn;
            display('turned right!');
        else
            if (side == 1)
                turn = turn/2;
                side = 0;
            end
            turn_to_target (serPort, (-1) * turn);
            total_angle_turned = total_angle_turned - turn;
            display('turned left!');
        end

        [com, roi] = camera();
    end
    total_angle_turned
end

function findDoor_old(serPort)

    [com, roi]  = camera()
    turn  = 16

    % line depends on which side the door is at
    

    if (com(1) < 160)
        side = 1;
    else
        side = 0;
    end

    while (abs(com(1) - 60) > 40)
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        if (turn == 1)
            break;
        end
        if(com(1) < 160)
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
    end

end

function line = determine_line(com_x, picture_width, ratio) %ratio > 0.5

    if (com_x < picture_width/2)
        line = (1 - ratio) * picture_width
    else
        line  = ratio * picture_width
    end
end

function ed= find_light (img)
    %img = imread ('im2.png');
    I = rgb2gray (img);
 
    % mask the bottom 1/3
    centerIndex = round (size(I,1) / 3 * 2);
    I(centerIndex:end,:) = cast (0, class(I));
 
    % do threasholding for lights
    bw = im2bw (I, 0.95);
    se = strel ('square', 7);
    ed = imerode (bw, se);
    % prop = ed;
    %s = regionprops (ed, 'centroid');
    %centroids = cat(1, s.Centroid);
 
    %imshow (ed)
    %hold on
    %plot(centroids(:,1), centroids(:,2), 'b*')
    %hold off

    %prop = s;
end

function tuneOrientation(serPort)

    turn = 16;

    tune_pos = findTunePos(serPort);
    if (tune_pos < 160)
        side = 1;
    else
        side = 0;
    end

    while (abs(tune_pos - 160) > 20)
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
        if(tune_pos < 160)
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

        tune_pos = findTunePos(serPort);
    end
end

function tune_pos = findTunePos(serPort)
    image = imread('http://192.168.0.100/img/snapshot.cgi?');
    imshow(image);
    ed = find_light(image);
    imshow(ed);
    prop_area = regionprops(ed, 'area');
    while (length(prop_area) == 0)
        display('!!!nothing here!!!')
        turnAngle(serPort, 0.025, 60);
        image = imread('http://192.168.0.100/img/snapshot.cgi?');
        imshow(image);
        ed = find_light(image);
    end
    %imshow (ed);
    display('found light after tuning');
    prop_area = regionprops(ed, 'area');
    length(prop_area)
    prop_centroid = regionprops(ed, 'centroid');
    areas = cat(1, prop_area.Area);
    areas
    [max_area, max_index] = max(areas);
    max_index
    centroids = cat(1, prop_centroid.Centroid);
    tune_pos = centroids(max_index, 1)
end