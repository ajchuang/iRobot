

function findDoor(serPort)

	com  = camera()
	turn  = 16

	% line depends on which side the door is at
	line = determine_line(com(1), 320, 2/3)

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
function line = determine_line(com_x, picture_width, ratio) %ration > 0.5
	if (com_x < picture_width/2)
		line = (1 - ratio) * picture_width
	else
		line  = ratio * picture_width
	end
end

function center_of_mass = COM(BW)
binaryImage = true(size(BW));
labeledImage = bwlabel(binaryImage);
measurements = regionprops(labeledImage, BW, 'WeightedCentroid');
center_of_mass = measurements.WeightedCentroid;
end

function test()

    serPort = RoombaInit (3);

    findDoor (serPort);
end

