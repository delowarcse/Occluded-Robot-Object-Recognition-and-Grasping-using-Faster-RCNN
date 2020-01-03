% Experiment on Faster R-CNN (Testing File)
% Date: 11.07.2018
%% Clear the table
clear;
close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device
set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)

%% Get image file
frame=getdata(obj,1);

  im(:,:,:)=frame(120:250,240:420,:);
  stop(obj)
test_im = im;
figure, imshow(test_im)

%% region properties
%measurements = regionprops(test_im, 'all')
grayImage = test_im;
[rows, columns, numberOfColorChannels] = size(test_im);
if numberOfColorChannels > 1
	% It's not really gray scale like we expected - it's color.
	% Convert it to gray scale by taking only the green channel.
	grayImage = grayImage(:, :, 2); % Take green channel.
end
% Display the image.
%subplot(2, 2, 1);
figure, imshow(grayImage, []);
%title('Original Grayscale Image', 'FontSize', fontSize, 'Interpreter', 'None');

% Let's compute and display the histogram.
% figure
% histogram(grayImage, 0:256);
% grid on;

% Threshold and binarize the image
binaryImage = grayImage > 110;
% Display the image.
%subplot(2, 2, 3);
figure, imshow(binaryImage, []);

% Label the image
labeledImage = bwlabel(binaryImage);
% Get the orientation
measurements = regionprops(labeledImage, 'Orientation', 'MajorAxisLength', 'Centroid');
allAngles = -[measurements.Orientation]


  stop(obj);