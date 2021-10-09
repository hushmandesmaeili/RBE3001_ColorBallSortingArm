
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
aboveBall = 40;
atBall = 10;

%bin locations
% yellowBin = [10 -180 70];
% redBin = [115 -150 70];
% greenBin = [10 180 70];
% orangeBin = [115 150 70];

%some different locations to maybe avoid singularities?
yellowBin = [145 -90 70];
orangeBin = [145 50 70];
redBin = [145, 50, 70];
greenBin = [145 90 70];

drop_location = 0;

SPEED = 0.2;    % mm/s

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

caughtException = false;
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
                robot.setQuinticTraj(robot.position([-80 20 0]), 2000);
%                 robot.interpolate_jp([-80 20 0], 2000);
                robot.openGripper();
%                 pause(1);
                tic
                lastState = state;
                state = MOVING;
                nextState = CHECKYELLOW;

%                   state = CHECKYELLOW;
                  
            case MOVING
                try
                    if (~robot.atEndPoint())
                        robot.trajMove(toc);
                    else
                        state = nextState;
                        nextState = NEXTSTATEERROR;
                    end
                catch exception
                    if strcmp(exception.identifier, 'Robot:bound') || strcmp(exception.identifier, 'Robot:xbound')
                        if strcmp(exception.identifier, 'Robot:bound'), caughtException = true; end
                        robot.setQuinticTraj([100 0 50], 2000);
                        nextState = lastState;
                        lastState = LASTSTATEERROR;
                        state = MOVING;
                        tic
                    else 
                        rethrow(exception);
                    end
                end

            % Move to ball
            case MOVETOBALL
                robot.setQuinticTraj(ballPos, 2000);
                tic
                robot.openGripper();
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
                 time = robot.getInterpolationTime(drop_location, SPEED);
                 robot.setQuinticTraj(drop_location, 2000);
                 tic
                 lastState = CHECKYELLOW;
                 state = MOVING;
                 nextState = PLACE;

            % Place ball    
            case PLACE
                robot.openGripper();
                state = RESET;

            % Check if yellow balls
            case CHECKYELLOW
                try
                    if(~caughtException)
                        if (cam.isColorPresent('y'))
                            ballPos = cam.ballPosition('y');
                            time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                            robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                            drop_location = yellowBin;
                            tic
                            lastState = state;
                            state = MOVING;
                            nextState = MOVETOBALL;
                            
                        else
                            state = CHECKRED;
                        end
                    else
                        time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                        drop_location = yellowBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                        caughtException = false;
                    end
                catch exception
                    if strcmp(exception.identifier, 'Camera:noBall')
                        state = RESET;
                    else
                        rethrow(exception);
                    end
                end
                
           % Check if red balls
            case CHECKRED
                try
                    if(~caughtException)
                        if (cam.isColorPresent('r'))
                            ballPos = cam.ballPosition('r');
                            time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                            robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                            drop_location = redBin;
                            tic
                            lastState = state;
                            state = MOVING;
                            nextState = MOVETOBALL;
                            
                        else
                            state = CHECKGREEN;
                        end
                    else
                        time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                        drop_location = redBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                        caughtException = false;
                    end
                catch exception
                    if strcmp(exception.identifier, 'Camera:noBall')
                        state = RESET;
                    else
                        rethrow(exception);
                    end
                end
                
            % Check if green balls
            case CHECKGREEN
                try
                    if(~caughtException)
                        if (cam.isColorPresent('g'))
                            ballPos = cam.ballPosition('g');
                            time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                            robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                            drop_location = greenBin;
                            tic
                            lastState = state;
                            state = MOVING;
                            nextState = MOVETOBALL;
                        else
                            state = CHECKORANGE;
                        end
                    else
                        time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                        drop_location = greenBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                        caughtException = false;
                    end
                catch exception
                    if strcmp(exception.identifier, 'Camera:noBall')
                        state = RESET;
                    else
                        rethrow(exception);
                    end
                end
                
           % Check if orange balls
            case CHECKORANGE
                try
                    if(~caughtException)
                        if (cam.isColorPresent('o'))
                            ballPos = cam.ballPosition('o');
                            time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                            robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                            drop_location = orangeBin;
                            tic
                            lastState = state;
                            state = MOVING;
                            nextState = MOVETOBALL;
                            
                        else
                            state = CHECKYELLOW;
                        end
                    else
                        time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
                        robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
                        drop_location = orangeBin;
                        tic
                        lastState = state;
                        state = MOVING;
                        nextState = MOVETOBALL;
                        caughtException = false;
                    end
                catch exception
                    if strcmp(exception.identifier, 'Camera:noBall')
                        state = RESET;
                    else
                        rethrow(exception);
                    end
                end
                
            %next state error condition
            case NEXTSTATEERROR
                error("next state not set");
                
            %last state error condition
            case LASTSTATEERROR
                error("last state not set");

        end
    end

    
catch exception
    robot.openGripper();
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
% 
% function out = ballHelper(ballPos, bin, state, aboveBall, SPEED)
%     time = robot.getInterpolationTime([ballPos(1) ballPos(2) aboveBall], SPEED);
%     robot.setQuinticTraj([ballPos(1) ballPos(2) aboveBall], 2000);
%     drop_location = bin;
%     tic
%     lastState = state;
%     state = MOVING;
%     nextState = MOVETOBALL;
%     out = [drop_location lastState state nextState];
% end