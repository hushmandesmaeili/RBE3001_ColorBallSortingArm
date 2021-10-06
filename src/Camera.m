classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = true;  
        DEBUG_BALLDETECTION = false;
        
        % Properties
        params;
        cam;
        undist_cam_params;
        cam_pose;
        cam_imajl;
    end
    
    properties (Access = private)
        camHeight = 278; %mm
        ballRad = 10; %mm
        camx = 200; %mm
        
        T0Check = [0 1 0 50;
           1 0 0 -100;
           0 0 -1 0;
           0 0 0 1];
        
        checkPos = [-25 225;
                 -75 100];
        camPos = zeros(2,5);
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_imajl, self.cam_pose] = self.getCameraPose();
            self.camPos(:, 1) = worldToImage(self.params.Intrinsics, self.cam_pose(1:3, 1:3), self.cam_pose(1:3, 4), [self.checkPos(1, 1) self.checkPos(2, 1) 0]);
            self.camPos(:, 2) = self.camPos(:, 1);
            self.camPos(:, 3) = worldToImage(self.params.Intrinsics, self.cam_pose(1:3, 1:3), self.cam_pose(1:3, 4), [self.checkPos(1, 1) self.checkPos(2, 2) 0]);
            self.camPos(:, 4) = worldToImage(self.params.Intrinsics, self.cam_pose(1:3, 1:3), self.cam_pose(1:3, 4), [self.checkPos(1, 2) self.checkPos(2, 2) 0]);
            self.camPos(:, 5) = worldToImage(self.params.Intrinsics, self.cam_pose(1:3, 1:3), self.cam_pose(1:3, 4), [self.checkPos(1, 2) self.checkPos(2, 1) 0]);
        end
        

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALOBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
%                 camcalib; % Change this if you are using a different calibration script
                camcalib2;
                params = cameraParams;
                disp("This is REEE Frogg:")
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camerea calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.undist_cam_params, 'OutputView', 'full');
        end

        
        function [newIs, pose] = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!
            
            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!
            
            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
%             % 2. Undistort Image based on param
            
            self.undist_cam_params = self.params.Intrinsics;
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            self.params.Intrinsics = newIs;
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, self.params.Intrinsics);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
                
            if self.POSE_PLOT
                
                figure(10)
                newImgPoints = [];
                c = 1;
                for i = 0:25:200
                    for j = 0:25:75
                        newImgPoints(c,1) = i;
                        newImgPoints(c,2) = j;
                        newImgPoints(c,3) = 0;
                        c = c + 1;
                    end
                end
                
                newGridPoints = worldToImage(newIs, R, t, newImgPoints);

                axesPoints = worldToImage(newIs, R, t, [0 0 0; 0 50 0; 50 0 0]);
%                 fake = worldToImage(newIs, R, t, [125 50 0])
%                 fake2 = pointsToWorld(newIs, R, t, fake)

                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
%                 hold on 
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                viscircles(newGridPoints, ones(length(newGridPoints),1)*5);
%                 hold off
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end     
            
            
        end
        
        function [newIs, pose] = getRealCameraPose(self)
                newIs = self.cam_imajl;
                pose = self.cam_pose;
        end
        
        function imgOut = boardMask(self, img)
            maskOut = poly2mask(self.camPos(1,:), self.camPos(2,:), 287, 360);
            imgOut = maskOut.*img;
        end
        
        function imgOut = colorMask(self, img, char)
           
           switch char
               case 'g'
                   [imgOut, w] = greenMask(img);
               case 'y'
                   [imgOut, w] = yellowMask(img);
               case 'o'
                   [imgOut, w] = orangeMask(img);
               case 'r'
                   [imgOut, w] = redMask(img);
           end
           
           imgOut = self.boardMask(imgOut);
           imgOut = imfill(imgOut, 'holes');
           imgOut = bwareaopen(imgOut, 200);
        end
        
        function point = camToRobot(self, img_point)
           g = pointsToWorld(self.cam_imajl, self.cam_pose(1:3, 1:3), self.cam_pose(1:3, 4), img_point);
           c = [g(1); g(2); 0; 1]; %position in terms of checkerboard
           point = self.T0Check*c;
        end
        
        function c = findCentroid(self, char)
            inImg = self.getImage();
            out = self.colorMask(inImg, char);
            props = regionprops(out);
            c_img = props.Centroid;
            c = self.camToRobot(c_img);
%             imshowpair(inImg, out);
        end
        
        %returns xyz position of center of ball;
        function a = ballPosition(self, char)
            centroid = self.findCentroid(char);
            camDist = sqrt((self.camx - centroid(1))^2 + centroid(2)^2);
            diff = (camDist*self.ballRad)/self.camHeight;
            
            p = centroid;
            xpb = p(1); %x-distance from projected point to base
            ypb = p(2); %y-distance from projected point to base
            %hpb = sqrt(ybp^2+xbp^2); %distance from projected point to base (hypotenuse)
            
            distcb = 200; %distance from base to camera
            
            xpc = distcb-xpb; %x distance from projected point to camera
            hpc = sqrt(xpc^2+ypb^2); %distance from projected point to camera (hypotenuse)
            
            hac = hpc-diff; %distance from actual point to camera;
            ya = (ypb*hac)/hpc; %y distance from a to base (same as distance from a to camera)
            xac = ((xpc*hac)/hpc); %x distance from base to actual point
            xa = self.camx - xac;
            
            a = [xa ya self.ballRad];
        end
        
        % Checks to see if color is present in board
        % Takes in color choice as parameter, returns true/false depending 
        % if color is present in board
        function colorPresent = isColorPresent(self, char)
            inImg = self.getImage();
            out = self.colorMask(inImg, char);
            
            colorPresent = ismember(1, out);
        end
    end
end