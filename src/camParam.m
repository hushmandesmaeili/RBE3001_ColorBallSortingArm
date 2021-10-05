% Auto-generated by cameraCalibrator app on 29-Sep-2021
%-------------------------------------------------------

%disp("REEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
% Define images to process
imageFileNames = {'/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image24.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image27.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image28.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image30.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image33.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image34.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image36.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image46.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image57.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image58.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image59.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image60.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image61.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image63.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image65.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image66.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image67.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image68.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image69.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image72.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image77.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image78.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image79.png',...
    '/home/jorobinson/RBE3001_Matlab11/camera_calibration/Image80.png',...
    };
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 25;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera using fisheye parameters
[cameraParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(imagePoints, worldPoints, ...
    [mrows, ncols], ...
    'EstimateAlignment', false, ...
    'WorldUnits', 'millimeters');

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortFisheyeImage(originalImage, cameraParams.Intrinsics);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
