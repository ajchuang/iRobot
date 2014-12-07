%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2014
%
% Homework 4
%
% Team number: 10
% Team leader: Brian Slakter (bjs2135)
% Team members: Sinjihn Smith (sss2208), Sheng Qian(sq2168)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw4_team_100(serPort)
    serPort = RoombaInit (3);
    %paste path here - calculated from shortest path algorithm in Java code
    path=[[0.58, -3.107];[0.5332479999999999, -2.4277800000000003];[0.5332479999999999, -1.58778];[0.6243989999999999, -0.187911];[0.6243989999999999, 0.6520889999999999];[-0.217342, 4.67459];[-0.217342, 5.514589999999999];[-0.03, 10.657]];
    
    %Initialize variables
    maxFwdVel= 0.3;         % Max allowable forward velocity 
    maxDuration=1200000;    % Max Duration
    angularVeloRotate = 0.2;% Angular velocity for rotation
    angle_e = 0;            % Initial angle set to 0, only updates after first bump
    tStart= tic;            % Time limit marker
    distThresh = 0.05;      % Allowed range for distance traveled vs desired
    angleThreshold=0.02;    % Allowed range for real angle to differ from desired
    distance=0;             % Distance between 2 points on path
    xPos=0.58;              % Starting x coordinate - location of start
    yPos=-3.107;            % Starting y coordinate - location of start
    angle=0;                % Start with angle set to zero
    
    % Initialize sensors
    AngleSensorRoomba(serPort);
    DistanceSensorRoomba(serPort);
    
    % Determine length of path from start to goal
    n=length(path);
    % initialize distance list and turn list 
    Dis=[];
    AngleT=[];
    
    % preprocessing loop - calculate angles and distances to travel
    for i=1:n-1
        
        xpos=path(i,1);
        ypos=path(i,2);
        xnext=path(i+1,1);
        ynext=path(i+1,2);
        disX=xnext-xpos;
        disY=ynext-ypos;
        angleT=atan(disY/disX);
        
        % if above 360 degrees, mod to bring below
        angle_e=mod(angle_e,2*pi);
        
        % if x bigger than zero, than point will be on right - then we want
        % to turn clockwise, or a more negative angle
        % otherwise will be on left
        if disX>=0
            angleT=angleT-pi/2;
        else
            angleT=angleT+pi/2;
        end
        
        % angle relative to origin coordinate frame
        angleT=angleT-angle_e;
        % make the turned angle smallest
        angleT=mod(angleT,2*pi);
        
        % Rid unnecessary turning
        if angleT>pi
            angleT=angleT-2*pi;
        end
        
        %update angle to save for next point
        angle_e=angleT+angle_e;
        
        
        % add angle to list
        AngleT=[AngleT angleT];
        
        % calculate distance and add to list
        distance=sqrt((xnext-xpos)^2+(ynext-ypos)^2);
        Dis=[Dis distance];
    end
    
    % Uncomment this if you would like to see plot of X-Y in real time 
    % Must also uncomment plot and hold on lines in updatePosition function
    %figure(1);
    
    % loop driving roomba during race
    while toc(tStart) < maxDuration
        %cycle through intervals
        for i=1:n-1
            
            % make sure orientation of turn is appropriate
            angularVeloRotateNew=angularVeloRotate*sign(AngleT(i));
            
            % initialization of temporary angle and distance
            % will be compared to desired value and incremented until
            % within appropriate threshold
            angle_temp=0;
            distance_temp=0;
            
            % keep turning until we reach threshold
            while abs(angle_temp-AngleT(i))>=angleThreshold
               SetFwdVelAngVelCreate(serPort, 0, angularVeloRotateNew);
               pause(0.01);
               [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
               angle_temp=angle_temp+D_angle;
            end
            
            % keep moving forward until we've reached distance threshold
            while abs(distance_temp-Dis(i))>=distThresh
                SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
                pause(0.01);
                [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xPos, yPos, angle);
                distance_temp=distance_temp+D_distance;
            end

        end
        
        % stop roomba and exit loop
        SetFwdVelAngVelCreate(serPort, 0, 0);
        break
    end
end

% Function to update x and y positions relative to origin
% D_distance and D_angle represent the incremental increase in angle or
% distance as we turn or move forward
function [xPos, yPos, angle,D_distance,D_angle] = updatePosition(serPort, xold, yold, angleold)
    
% Angle will be cumulitave, and relative to first bump
    D_angle=AngleSensorRoomba(serPort);
    angle = angleold +D_angle;
    
    % Distance since last call
    D_distance = DistanceSensorRoomba(serPort);
    % Update x and y positons based on updated angle and distance reading
    xPos = xold - D_distance*sin(angle);
    yPos = yold + D_distance*cos(angle);
    % Uncomment these two lines for plotting
    % plot(xPos,yPos,'o');
    % hold on;
end