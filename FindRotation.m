% how to detect the value angle of rotation object ?

clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 20;

% Check that user has the Image Processing Toolbox installed.
hasIPT = license('test', 'image_toolbox');   % license('test','Statistics_toolbox'), license('test','Signal_toolbox')
if ~hasIPT
	% User does not have the toolbox installed.
	message = sprintf('Sorry, but you do not seem to have the Image Processing Toolbox.\nDo you want to try to continue anyway?');
	reply = questdlg(message, 'Toolbox missing', 'Yes', 'No', 'Yes');
	if strcmpi(reply, 'No')
		% User said No, so exit.
		return;
	end
end

%===============================================================================
% Read in gray scale demo image.
folder = pwd
baseFileName = '1.jpg';
% Get the full filename, with path prepended.
fullFileName = fullfile(folder, baseFileName);
% Check if file exists.
if ~exist(fullFileName, 'file')
	% File doesn't exist -- didn't find it there.  Check the search path for it.
	fullFileNameOnSearchPath = baseFileName; % No path this time.
	if ~exist(fullFileNameOnSearchPath, 'file')
		% Still didn't find it.  Alert user.
		errorMessage = sprintf('Error: %s does not exist in the search path folders.', fullFileName);
		uiwait(warndlg(errorMessage));
		return;
	end
end
grayImage = imread(fullFileName);
% Get the dimensions of the image.  
% numberOfColorBands should be = 1.
[rows, columns, numberOfColorChannels] = size(grayImage);
if numberOfColorChannels > 1
	% It's not really gray scale like we expected - it's color.
	% Convert it to gray scale by taking only the green channel.
	grayImage = grayImage(:, :, 2); % Take green channel.
end
% Display the image.
subplot(2, 2, 1);
imshow(grayImage, []);
title('Original Grayscale Image', 'FontSize', fontSize, 'Interpreter', 'None');

% Set up figure properties:
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% Get rid of tool bar and pulldown menus that are along top of figure.
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
% Give a name to the title bar.
set(gcf, 'Name', 'Demo by ImageAnalyst', 'NumberTitle', 'Off') 

% Let's compute and display the histogram.
subplot(2, 2, 2); 
histogram(grayImage, 0:256);
grid on;
title('Histogram of original image', 'FontSize', fontSize, 'Interpreter', 'None');
xlabel('Gray Level', 'FontSize', fontSize);
ylabel('Pixel Count', 'FontSize', fontSize);
xlim([0 255]); % Scale x axis manually.

% Threshold and binarize the image
binaryImage = grayImage > 128;
% Display the image.
subplot(2, 2, 3);
imshow(binaryImage, []);
axis on;
title('Binary Image', 'FontSize', fontSize, 'Interpreter', 'None');

% Label the image
labeledImage = bwlabel(binaryImage);
% Get the orientation
measurements = regionprops(labeledImage, 'Orientation', 'MajorAxisLength', 'Centroid');
allAngles = -[measurements.Orientation]
hold on;
for k = 1 : length(measurements)
	fprintf('For blob #%d, the angle = %.4f\n', k, allAngles(k));
	xCenter = measurements(k).Centroid(1);
	yCenter = measurements(k).Centroid(2);
	% Plot centroids.
	plot(xCenter, yCenter, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
	% Determine endpoints
	axisRadius = measurements(k).MajorAxisLength / 2;
	x1 = xCenter + axisRadius * cosd(allAngles(k));
	x2 = xCenter - axisRadius * cosd(allAngles(k));
	y1 = yCenter + axisRadius * sind(allAngles(k));
	y2 = yCenter - axisRadius * sind(allAngles(k));
	fprintf('x1 = %.2f, y1 = %.2f, x2 = %.2f, y2 = %.2f\n\n', x1, y1, x2, y2);
	plot([x1, x2], [y1, y2], 'r-', 'LineWidth', 2);
end
