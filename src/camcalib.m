% Camera Calibration Script
% Most of this script is generated, but a few things were modified to make it 
% nicer to run. For example, a DEBUG flag was added so the script will not
% generate figures unless absolutely needed!


if exist('DEBUG', 'var') ~= 1
    DEBUG = true;
end

% Define images to process
% Note: This WILL be different, make sure you replace these with your pictures before
% imageFileNames = {'/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image1.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image10.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image11.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image12.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image13.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image14.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image15.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image16.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image17.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image18.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image19.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image2.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image20.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image3.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image4.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image5.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image6.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image7.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image8.png',...
% '/media/Data/alex/Documents/wpi/rbe-3001/matlab/camera_calibration/lab_arm/Image9.png',...
% };

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

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

if DEBUG
    
    % View reprojection errors
    h1=figure; showReprojectionErrors(cameraParams);

    % Visualize pattern locations
    h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

    % Display parameter estimation errors
    displayErrors(estimationErrors, cameraParams);
    
end
