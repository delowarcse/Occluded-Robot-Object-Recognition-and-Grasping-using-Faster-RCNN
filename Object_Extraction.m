clear;
close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device

set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)

count = 0;

% while 1
%for k = 1:1
frame=getdata(obj,1);
%main = strcat('MainIMAGE_06_00',num2str(1),'.jpg');
%imwrite(frame,main);

%     for i = 90:380 %30:200
%         for j = 130:520
%             im(i,j,:) = frame(i,j,:);
%         end
%     end
im(:,:,:)=frame(120:250,240:420,:);
imshow(im);

frame1 = rgb2gray(im);

% Detecting a Cell Using Image Segmentation
% Step 2: Detect Entire Cell
[~, threshold] = edge(frame1, 'Canny');
fudgeFactor = 7; %5 not bad, 7,
BWs = edge(frame1,'Canny', threshold * fudgeFactor);
%     BWs = edge(frame1,'Canny', threshold);
%     imshow(BWs), title('binary gradient mask');

% Step 3: Dilate the Image
se90 = strel('line', 3, 90); %Create morphological structuring element (STREL)
se0 = strel('line', 3, 0); % strel('line', Length, Degree)

BWsdil = imdilate(BWs, [se90 se0]);
%  imshow(BWsdil), title('dilated gradient mask')

%  Step 4: Fill Interior Gaps
BWdfill = imfill(BWsdil, 'holes');
%     imshow(BWdfill); title('binary image with filled holes');

%Step 5: Remove Connected Objects on Border
BWnobord = imclearborder(BWdfill, 8);
%     imshow(BWnobord), title('cleared border image');

%     Step 6: Smoothen the Object
seD = strel('diamond',1);
BWfinal = imerode(BWnobord,seD);
BWfinal = imerode(BWfinal,seD);
%     imshow(BWfinal), title('segmented image');
%     hold on

%     An alternate method for displaying
%     the segmented object would be to place an outline
%     around the segmented cell.
%     The outline is created by the bwperim function.
BWoutline = bwperim(BWfinal);
Segout = frame1;
Segout(BWoutline) = 255;
figure,imshow(Segout), title('outlined original image');
hold on

% Remove all object containing fewer than 30 pixels
bwao = bwareaopen(BWfinal,30);

% Label the binary image and computer the centroid and center of mass.
% labeledImage = bwlabel(BWfinal);
[bw,Ne] = bwlabel(bwao, 8); % bw is label matrix and Ne is connected object

% traces the exterior boundaries of objects
% noholes is search only for object (parent and child) boundaries
[B,L] = bwboundaries(bw,'noholes');

loop = length(B);

%     measurements = regionprops(L,frame1, ...
%         'BoundingBox', 'Centroid', 'WeightedCentroid', 'Perimeter','Orientation','PixelValues');
measurements = regionprops(L,frame1,'all');

%     count = count+1
centroid = [];
orientation = [];
box = [];
val = [];

for object=1:loop
    box(object,:) = measurements(object).BoundingBox;
    centroid(object,:) = measurements(object).Centroid
    centerOfMass(object,:) = measurements(object).WeightedCentroid;
    perimeter(object,:) = measurements(object).Perimeter;
    orientation(object,:) = measurements(object).Orientation
    plot(centroid(object,1),centroid(object,2),'o');
    %rectangle('Position',[centroid(object,1)-17,centroid(object,2)-18,36,36]);
end

%end
stop(obj)
