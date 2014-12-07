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
    
    serPort = RoombaInit (3);

    while (true) 
        
        tuneOrientation (serPort);
        
        com = camera ();
        line = determine_line (com(1), 320, 2/3);
        findDoor (serPort, line);
        SetFwdVelAngVelCreate (serPort, 0.22, 0.0);
        
        [bRight bLeft x y z bCenter] = BumpsWheelDropsSensorsRoomba (serPort);

        if (bRight | bCenter | bLeft)
            display ('Home!');
            SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
            return;
        end
    end
    
    findDoor (serPort);
end

function stats = correctPath (serPort)

    com = camera ();
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


function center_of_mass = camera()

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

function center_of_mass = COM(BW)
binaryImage = true(size(BW));
labeledImage = bwlabel(binaryImage);
measurements = regionprops(labeledImage, BW, 'WeightedCentroid');
center_of_mass = measurements.WeightedCentroid;
end

function findDoor(serPort, line)

    com  = camera()
    turn  = 16

    % line depends on which side the door is at
    

    if (com(1) < line)
        side = 1;
    else
        side = 0;
    end

    while (abs(com(1) - line) > 20)
        SetFwdVelAngVelCreate (serPort, 0.0, 0.0);
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

        com = camera();
    end

end

function line = determine_line(com_x, picture_width, ratio) %ratio > 0.5

    if (com_x < picture_width/2)
        line = (1 - ratio) * picture_width
    else
        line  = ratio * picture_width
    end
end

function prop= find_light (img)
    %img = imread ('im2.png');
    I = rgb2gray (img);
 
    % mask the bottom 1/3
    centerIndex = round (size(I,1) / 3 * 2);
    I(centerIndex:end,:) = cast (0, class(I));
 
    % do threasholding for lights
    bw = im2bw (I, 0.97);
    se = strel ('square', 7);
    ed = imerode (bw, se);
    s  = regionprops (ed, 'centroid');
    centroids = cat(1, s.Centroid);
 
    %imshow (ed)
    %hold on
    %plot(centroids(:,1), centroids(:,2), 'b*')
    %hold off
    
    prop = s;
end

function tuneOrientation(serPort)
    
    tune_pos = findTunePos();
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

        tune_pos = findTunePos();
    end
end

function tune_pos = findTunePos()
    image = imread('http://192.168.0.101/img/snapshot.cgi?');
    prop = find_light(image);
    (max_area, max_index) = max(prop.Area);
    centroids = cat(1, prop.Centroid;)
    tune_pos = centroids(max_index, 1)
end

