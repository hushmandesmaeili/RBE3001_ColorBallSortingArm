
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


%enum lol
RESET = 0;
MOVING = 1;
MOVETOBALL = 2;
PICK = 3;
SLOWRISE = 4;
PLACE = 5;
CHECKYELLOW = 6;
CHECKRED = 7;
CHECKGREEN = 8;
CHECKORANGE =  9;
NEXTSTATEERROR = 10;
LASTSTATEERROR= 11;

state = RESET;
nextState = NEXTSTATEERROR;
lastState = LASTSTATEERROR;

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found ERROR!!! !");
    end
    
    
    while true
        switch state

            % Reset arm and gripper
            case RESET
%                 robot.setQuinticTraj([-80 20 0], 2000);
                robot.interpolate_jp([-80 20 0], 2000);
                robot.openGripper();
                pause(1);
%                 lastState = state;
%                 state = MOVING;
%                 nextState = CHECKYELLOW;

                  state = CHECKYELLOW;
                  
            case MOVING
                try
                    if (~robot.atEndPoint())
                        robot.trajMove(toc);
                    else
                        state = nextState;
                        nextState = NEXTSTATEERROR;
                    end
                catch exception
                    if strcmp(exception.identifier, 'Robot:bound')
                        robot.setQuinticTraj([100 0 50], 2000);
                        tic
                        nextState = lastState;
                        lastState = LASTSTATEERROR;
                        state = MOVING;
                    else
                        rethrow(exception);
                    end
                end

            % Move to ball
            case MOVETOBALL
                robot.setQuinticTraj(ballPos, 2000);
                tic
                lastState = state;
                state = MOVING;
                nextState = PICK;
            
            % Pick ball    
            case PICK
                 robot.closeGripper();
                 robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                 tic
                 lastState = state;
                 state = MOVING;
                 nextState = SLOWRISE;
                 
            % Slow rise     
            case SLOWRISE
                 robot.setQuinticTraj(drop_location, 3000);
                 tic
                 lastState = state;
                 state = MOVING;
                 nextState = PLACE;

            % Place ball    
            case PLACE
                robot.openGripper();
                state = RESET;

            % Check if yellow balls
            case CHECKYELLOW
                if (cam.isColorPresent('y'))
                    try
                        ballPos = cam.ballPosition('y');
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                        drop_location = yellowBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                    catch exception
                        if strcmp(exception.identifier, 'Camera:noball')
                            state = RESET;
                        else
                            rethrow(exception);
                        end
                    end
                else
                    state = CHECKRED;
                end
                
           % Check if red balls
            case CHECKRED
                if (cam.isColorPresent('r'))
                    try
                        ballPos = cam.ballPosition('r');
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                        drop_location = redBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                    catch exception
                        if strcmp(exception.identifier, 'Camera:noball')
                            state = RESET;
                        else
                            rethrow(exception);
                        end
                    end
                else
                    state = CHECKGREEN;
                end
                
            % Check if green balls
            case CHECKGREEN
                if (cam.isColorPresent('g'))
                    try
                        ballPos = cam.ballPosition('g');
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                        drop_location = redBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                    catch exception
                        if strcmp(exception.identifier, 'Camera:noball')
                            state = RESET;
                        else
                            rethrow(exception);
                        end
                    end
                else
                    state = CHECKORANGE;
                end
                
           % Check if orange balls
            case CHECKORANGE
                if (cam.isColorPresent('o'))
                    try
                        ballPos = cam.ballPosition('o');
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 3000);
                        drop_location = redBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                    catch exception
                        if strcmp(exception.identifier, 'Camera:noball')
                            state = RESET;
                        else
                            rethrow(exception);
                        end
                    end
                else
                    state = CHECKYELLOW;
                end
                
            %next state error condition
            case NEXTSTATEERROR
                error("next state not set");
                
            %last state error condition
            case LASTSTATEERROR
                error("last state not set");

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
