%% Clear the table
clear;
close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device
set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)
%% Load R-CNN
load lmFRC_SMObj_05072018_1
net = lmFRCNN;
%% Get image file
frame=getdata(obj,1);

  %im(:,:,:)=frame(86:300,151:500,:);
  im(:,:,:)=frame(120:245,235:415,:);
  stop(obj)
test_im = im;
img = im;
figure, imshow(test_im)
%figure
%stop(obj)
%test_im = imread(file_nam);
%% Detection
tic
[bboxes, score, label] = detect(net, test_im)
toc
%% region properties

%% Output image
figure;
ixx = 1;
for i=1:size(score)
    if score(i)>=0.5
        bbox(ixx,:) = bboxes(i,:);
        label_str{ixx} = char(string(label(i)));
        score_str{ixx} = char(string(score(i)));
        detectedImg = insertShape(test_im, 'Rectangle', bbox);
        Center(ixx,:) = round([bbox(i,1)+bbox(i,3)/2,bbox(i,2)+bbox(i,4)/2]);
        ixx = ixx+1;
        imshow(detectedImg)
%         pause
    end
end
%% region properties
%Center

if exist('bbox','var')
    outputImage = insertObjectAnnotation(test_im, 'rectangle', bbox, label_str,'FontSize', 14,'LineWidth',3);
    figure
    imshow(outputImage)
    %imtool(test_im)
    %title('Object Detected')
else
    figure
    imshow(test_im)    
    title('Nothing')
end


%% recognize the specifice object
label1 = cellstr(label) %convert categorical array to cell string array
score1 = score;

ixx1=1;
idx=[];
for j = 1:size(label1)
    if (strcmp(label1(j),'BK')==1)  % Requested object
        idx(ixx1) = j
        ixx1 = ixx1+1;
    end
end

if (isempty(idx)==1)
    %disp('The requested object does not exist');
    h = msgbox('The requested object does not exist', 'Warn','warn');
    return;
end

for k=1:size(idx,2)
    bbox2(k,:) = bbox(idx(k), :)
    label2(k) = label1(idx(k))
    score2(k) = score1(idx(k))
end
detectedImg = insertObjectAnnotation(img, 'rectangle', bbox2, label2,'FontSize', 14,'LineWidth',3);
figure
imshow(detectedImg)

% pick-up the highest score object
[score3, ind1] = max(score2);
bbox3 = bbox2(ind1,:);
label3 = label2(ind1);
detectedImg1 = insertObjectAnnotation(img, 'rectangle', bbox3, label3,'FontSize', 14,'LineWidth',3);
figure
imshow(detectedImg1)

%% Robot Grasping
Center = round([bbox3(1)+bbox3(3)/2,bbox3(2)+bbox3(4)/2]);
centroid = Center
pause
%% Robot grasping
armset;

% n = length(Center)
% for i = 1:n
    
    % Finding object position of X w.r.t robot
    ax =308.8193;
    %bx = 15; % to correct the robot position
    ax1 = 1.3962;
    centroid(2);
    %x = ax1*centroid(i,2)+ax+bx;
    x = ax1*centroid(2)+ax;
    
    % Finding object position of X w.r.t robot
    ay = -135.2372;
    %by = -10; % to correct the robot position
    ay1 = 1.4581;
    centroid(1);
    %y = ay1*centroid(i,1)+ay+by;
    y = ay1*centroid(1)+ay;
    
    %% Arm set

X = num2str(x);
Y = num2str(y);
Z = num2str(220);
Rx = num2str(-180);
Ry = num2str(0);
Rz = num2str(180);
fig = num2str(5);

P2 = strcat('(',X,',',Y,',',Z,',',Rx,',',Ry,',',Rz,',',fig,')');
rob.Move(1,P2);

pause