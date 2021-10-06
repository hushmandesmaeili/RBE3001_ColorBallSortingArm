
%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
close all;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);

% cam = Camera();
% save lab5cam cam
load lab5cam cam

cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [150, -50, 11];
green_place = [150, 50, 11]; 
pink_place = [75, -125, 11];
yellow_place = [75, 125, 11];

%setpoint ball heights
aboveBall = 35;
atBall = 10;

%bin locations
yellowBin = [10 -180 70];
redBin = [115 -150 70];
greenBin = [10 180 70];
orangeBin = [115 150 70];
drop_location = 0;

state = 0;
nextState = 5;

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found ERROR!!! !");
    end
    
    
    while true
        switch state

            % Reset arm and gripper
            case 0
                robot.interpolate_jp([-80 20 0], 2000);
                robot.openGripper();
                pause(1);
                state = nextState;

            % Move to ball
            case 1
                if (~robot.atEndPoint())
                    robot.trajMove(toc);
                else
                    robot.setQuinticTraj(ballPos, 2000);
                    tic
                    state = 2;
                end
            
            % Pick ball    
            case 2
                 if (~robot.atEndPoint())
                    robot.trajMove(toc);
                 else
                     robot.closeGripper();
                     robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                     tic
                     state = 3;
                 end
                 
            % Slow rise     
            case 3
                if (~robot.atEndPoint())
                    robot.trajMove(toc);
                else
                     robot.setQuinticTraj(drop_location, 3000);
                     tic
                     state = 4;
                 end

            % Place ball    
            case 4
                if (~robot.atEndPoint())
                    robot.trajMove(toc);
                else
                    robot.openGripper();
                    state = 0;
                    nextState = 5;
                end

            % Check if yellow balls
            case 5
                if (cam.isColorPresent('y'))
                    ballPos = cam.ballPosition('y');
                    robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                    drop_location = yellowBin;
                    tic
                    state = 1;
                else
                    state = 6;
                end
                
           % Check if red balls
            case 6
                if (cam.isColorPresent('r'))
                    ballPos = cam.ballPosition('r');
                    robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                    drop_location = redBin;
                    tic
                    state = 1;
                else
                    state = 7;
                end
                
            % Check if green balls
            case 7
                if (cam.isColorPresent('g'))
                    ballPos = cam.ballPosition('g');
                    robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                    drop_location = greenBin;
                    tic
                    state = 1;
                else
                    state = 8;
                end
                
           % Check if orange balls
            case 8
                if (cam.isColorPresent('o'))
                    ballPos = cam.ballPosition('o');
                    robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                    drop_location = orangeBin;
                    tic
                    state = 1;
                else
                    state = 5;
                end

        end
    end

%     while true
%         robot.interpolate_jp([-80 20 0], 2000);
%         robot.openGripper();
%         pause(2);
%         
%         ballPos = cam.ballPosition('y');
% 
%         %PICK
%         robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
%         tic
%         while ~robot.atEndPoint()
%             robot.trajMove(toc);
%         end
% 
%         robot.setQuinticTraj(ballPos, 2000);
%         tic
%         while ~robot.atEndPoint()
%             robot.trajMove(toc);
%         end
%         robot.closeGripper();
% 
%     %     pos = robot.currPosition()
%     %     disp(ballPos-pos);
%     %     disp(norm(ballPos - pos));
% 
%         %PLACE
%         robot.setQuinticTraj(yellowBin, 3001);
%         tic
%         while ~robot.atEndPoint()
%             robot.trajMove(toc)
%         end
%         robot.openGripper();
%     end
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end


%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
