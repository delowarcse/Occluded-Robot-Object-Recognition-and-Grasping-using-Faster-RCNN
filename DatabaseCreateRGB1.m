clear;
close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device

set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)

count = 0;

% while 1
for k = 1:1
    frame=getdata(obj,1);
    
    figure, imshow(frame);
    %     main = strcat('MainIMAGE_01_00',num2str(1),'.jpg');
    %     imwrite(frame,main);
    
    for i = 80:380
        for j = 130:520
            im(i,j,:) = frame(i,j,:);
        end
    end
    
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
    figure,imshow(Segout);% title('outlined original image');
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
        centroid(object,:) = measurements(object).Centroid;
        centerOfMass(object,:) = measurements(object).WeightedCentroid;
        perimeter(object,:) = measurements(object).Perimeter;
        orientation(object,:) = measurements(object).Orientation;
        % nn(object, :)=frame1(centroid(object):max(r),min(c):max(c));
        %pixelval(object,:) = measurements(object).PixelValues
        plot(centroid(object,1),centroid(object,2),'o');
        %pause
        
    end
    
    % Object Extraction
    for n=1:Ne
        
        i = 6;

%         % object extraction with respect to centroid, 28x28 pixels
%         n1(:,:,:) = im(round(centroid(n,2))-13:round(centroid(n,2))+14,round(centroid(n,1))-13:round(centroid(n,1))+14,:);
%         nn1 = strcat('Obj_04_28_00',num2str(i),'.jpg'); %automatically save images for train
% %         nn1 = strcat('Test_06_28_0',num2str(i),'.jpg'); %automatically save images for test
%         imwrite(n1,nn1); nn1;
%         figure,imshow(n1);
%         
        % object extraction with respect to centroid, 36x36 pixels
        n2(:,:,:) = im(round(centroid(n,2))-17:round(centroid(n,2))+18,round(centroid(n,1))-17:round(centroid(n,1))+18,:);
        nn2 = strcat('Obj_04_36_00',num2str(i),'.jpg'); %automatically save images for train
%         nn2 = strcat('Test_06_36_0',num2str(i),'.jpg'); %automatically save images for test
        imwrite(n2,nn2);nn2;
        figure,imshow(n2);
        
%         % object extraction with respect to centroid, 49x49 pixels
%         n3(:,:,:) = im(round(centroid(n,2))-24:round(centroid(n,2))+24,round(centroid(n,1))-24:round(centroid(n,1))+24,:);
%         nn3 = strcat('Obj_04_49_00',num2str(i),'.jpg'); %automatically save images for train
% %         nn3 = strcat('Test_06_49_0',num2str(i),'.jpg'); %automatically save images for test
%         imwrite(n3,nn3);nn3;
%         figure,imshow(n3);
%         
%         % object extraction with respect to centroid, 81x81 pixels
%         n4(:,:,:) = im( round(centroid(n,2))-40:round(centroid(n,2))+40, round(centroid(n,1))-40:round(centroid(n,1))+40, :);
%         nn4 = strcat('Obj_04_81_00',num2str(i),'.jpg'); %automatically save images for train
% %         nn4 = strcat('Test_06_81_0',num2str(i),'.jpg'); %automatically save images for test
%         imwrite(n4,nn4);nn4;
%         figure,imshow(n4);
%         
%         [  nn1; nn2;  nn3;  nn4]
        pause(0.4);
    end
    
    flushdata(obj);
end
stop(obj);