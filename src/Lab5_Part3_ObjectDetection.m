
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

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    [Is, pose] = cam.getRealCameraPose()
%       [re, ree] = cam.getCameraPose()
%       disp(ree)
%     cam.cam_pose
    T0Check = [0 1 0 50;
               1 0 0 -100;
               0 0 -1 0;
               0 0 0 1];
%    TCheck0 = [0 1 0 100;
%               1 0 0 -50;
%               0 0 -1 0;
%               0 0 0 1];
        
%     T0Cam = T0Check * pose

        %relate image points to base frame
       i = [135 160];
       g = pointsToWorld(Is, pose(1:3, 1:3), pose(1:3, 4), i);
       c = [g(1); g(2); 0; 1]; %position in terms of checkerboard
       p = T0Check*c;

%     inImage = snapshot(cam.cam);
%     [b, w] = greenMask(inImage);
% BW = bwareaopen(b, 90);
% imshow(BW);

    %find centroid of yellow ball
    inImg = cam.getImage();
    out = cam.colorMask(inImg, 'y');
    props = regionprops(out);
    centroid = props.Centroid
    imshowpair(inImg, out);

catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
