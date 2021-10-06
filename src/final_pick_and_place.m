
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
% load lab5cam cam

cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [150, -50, 11];
green_place = [150, 50, 11];
pink_place = [75, -125, 11];
yellow_place = [75, 125, 11];

%setpoint ball heights
aboveBall = 35;
atBall = 10;

%bin vocations
yellowBin = [0 -190 70];

%% Main Loop
try
    % Set up camera
%     if cam.params == 0
%         error("No camera parameters found!");
%     end

%     p = cam.camToRobot(centroid);

    robot.interpolate_jp([0 0 0], 2000);
    robot.openGripper();
    pause(2);
    
    %PICK
    robot.setQuinticTraj([75 -50 aboveBall], 3000);
    tic
    while ~robot.atEndPoint()
        robot.trajMove(toc);
    end
    
    robot.setQuinticTraj([75 -50 atBall], 2000);
    tic
    while ~robot.atEndPoint()
        robot.trajMove(toc);
    end
    
    robot.closeGripper();
    
    %PLACE
    robot.setQuinticTraj(yellowBin, 3001);
    tic
    while ~robot.atEndPoint()
        robot.trajMove(toc)
    end
    robot.openGripper();


catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end


%% Shutdown Procedure
robot.shutdown()
%cam.shutdown()
