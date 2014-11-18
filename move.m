


function move_to_next_pt(pts)
current_ang = 0;
x = pts(:,1);
y = pts(:,2);
for i = 1:length(x)-1
    theta = atan2d((x(i+1) - x(i)),(y(i+1) - y(i)));
    turn_ang = theta - current_ang;
    current_ang = current_ang + turn_ang;
    dist = sqrt((y(i+1) - y(i))^2 + (x(i+1) - x(i))^2);
    turnAngle (serPort, .15, turn_ang)
    travelDist (serPort, .2, dist)
end
end