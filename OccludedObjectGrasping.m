function varargout = OccludedObjectGrasping(varargin)
% OCCLUDEDOBJECTGRASPING MATLAB code for OccludedObjectGrasping.fig
%      OCCLUDEDOBJECTGRASPING, by itself, creates a new OCCLUDEDOBJECTGRASPING or raises the existing
%      singleton*.
%
%      H = OCCLUDEDOBJECTGRASPING returns the handle to a new OCCLUDEDOBJECTGRASPING or the handle to
%      the existing singleton*.
%
%      OCCLUDEDOBJECTGRASPING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OCCLUDEDOBJECTGRASPING.M with the given input arguments.
%
%      OCCLUDEDOBJECTGRASPING('Property','Value',...) creates a new OCCLUDEDOBJECTGRASPING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before OccludedObjectGrasping_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to OccludedObjectGrasping_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help OccludedObjectGrasping

% Last Modified by GUIDE v2.5 18-Sep-2018 15:34:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @OccludedObjectGrasping_OpeningFcn, ...
                   'gui_OutputFcn',  @OccludedObjectGrasping_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before OccludedObjectGrasping is made visible.
function OccludedObjectGrasping_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to OccludedObjectGrasping (see VARARGIN)

 %Showing object 1 in pushbutton 1
    [a1,map]=imread('Object_01.jpg');
    [r,c,d]=size(a1);
    x=ceil(r/150);
    y=ceil(c/150);
    g=a1(1:x:end,1:y:end,:);
    g(g==255)=5.5*255;
    %set(handles.axes1,'CData',g);
    axes(handles.axes1);
    imshow(g);
    
    % Showing object 2 in pushbutton 2
    [b1,map]=imread('Object_02.jpg');
    [r,c,d]=size(b1);
    x=ceil(r/150);
    y=ceil(c/150);
    g=b1(1:x:end,1:y:end,:);
    g(g==255)=5.5*255;
    %set(handles.axes1,'CData',g);
    axes(handles.axes2);
    imshow(g);
    
% Showing object 3 in pushbutton 3
    [c1,map]=imread('Object_03.jpg');
    [r,c,d]=size(c1);
    x=ceil(r/150);
    y=ceil(c/150);
    g=c1(1:x:end,1:y:end,:);
    g(g==255)=5.5*255;
    %set(handles.axes1,'CData',g);
    axes(handles.axes3);
    imshow(g);
    
    % Showing object 4 in pushbutton 4
    [d1,map]=imread('Object_04.jpg');
    [r,c,d]=size(d1);
    x=ceil(r/150);
    y=ceil(c/150);
    g=d1(1:x:end,1:y:end,:);
    g(g==255)=5.5*255;
    %set(handles.axes1,'CData',g);
    axes(handles.axes4);
    imshow(g);
    
    % Showing object 5 in pushbutton 5
    [e1,map]=imread('Object_06.jpg');
    [r,c,d]=size(e1);
    x=ceil(r/150);
    y=ceil(c/150);
    g=e1(1:x:end,1:y:end,:);
    g(g==255)=5.5*255;
    %set(handles.axes1,'CData',g);
    axes(handles.axes5);
    imshow(g);
    
%     % Showing object 6 in pushbutton 6
%     [f1,map]=imread('Object_06.jpg');
%     [r,c,d]=size(f1);
%     x=ceil(r/150);
%     y=ceil(c/150);
%     g=f1(1:x:end,1:y:end,:);
%     g(g==255)=5.5*255;
%     %set(handles.axes1,'CData',g);
%     axes(handles.axes6);
%     imshow(g);
    
% Choose default command line output for OccludedObjectGrasping
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes OccludedObjectGrasping wait for user response (see UIRESUME)
% uiwait(handles.figure1);

    
% --- Outputs from this function are returned to the command line.
function varargout = OccludedObjectGrasping_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Clear the table
clear;
%close all;
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
Center = round([bbox3(1,1)+bbox3(1,3)/2,bbox3(1,2)+bbox3(1,4)/2]);
centroid = Center
pause
%% Robot grasping
armset;

% n = length(Center)
% for i = 1:n
    
    % Finding object position of X w.r.t robot
    ax =308.8193;
    bx = -10; % to correct the robot position
    ax1 = 1.3962;
    centroid(2);
    %x = ax1*centroid(i,2)+ax+bx;
    x = ax1*centroid(2)+ax+bx;
    
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
Z = num2str(180);
Rx = num2str(-180);
Ry = num2str(0);
Rz = num2str(180);
fig = num2str(5);

P2 = strcat('(',X,',',Y,',',Z,',',Rx,',',Ry,',',Rz,',',fig,')');
rob.Move(1,P2);

pause
%end




% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Clear the table
clear;
%close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device
set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)
%% Load R-CNN
load lmFRC_SMObj_05072018_1
net = lmFRCNN;
%% Get image file
frame=getdata(obj,1);

%   im(:,:,:)=frame(86:300,151:500,:);
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
    if (strcmp(label1(j),'RD')==1)  % Requested object
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
Center = round([bbox3(1,1)+bbox3(1,3)/2,bbox3(1,2)+bbox3(1,4)/2]);
centroid = Center
%pause
%% Robot grasping
armset;

% n = length(Center)
% for i = 1:n
    
    % Finding object position of X w.r.t robot
    ax =308.8193;
    bx = 00; % to correct the robot position
    ax1 = 1.3962;
    centroid(2);
    %x = ax1*centroid(i,2)+ax+bx;
    x = ax1*centroid(2)+ax+bx;
    
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
Z = num2str(160);
Rx = num2str(-180);
Ry = num2str(0);
Rz = num2str(180);
fig = num2str(5);

P1 = strcat('(',X,',',Y,',',Z,',',Rx,',',Ry,',',Rz,',',fig,')');
rob.Move(1,P1);

     State = caoExt.Execute('get_BusyState');
        while State ~=0
            State = caoExt.Execute('get_BusyState');
        end
        
            % Move to the actual position (Z=135) of the object
            P2 = strcat('(',X,',',Y,',','150',',',Rx,',',Ry,',',Rz,',',fig,')');
            rob.Move(1,P2);
            
        State = caoExt.Execute('get_BusyState');
        while State ~=0
            State = caoExt.Execute('get_BusyState');
        end
        
        % Chuck the object, pick-up
        caoExt.Execute('Chuck',1);
        %pause
        
        State = caoExt.Execute('get_BusyState');
        while State ~=0
            State = caoExt.Execute('get_BusyState');
        end
        
        P4 = strcat('(',X,',',Y,',','170',',',Rx,',',Ry,',',Rz,',',fig,')');
        rob.Move(1,P4);
        %pause
        
        State = caoExt.Execute('get_BusyState');
        while State ~=0
            State = caoExt.Execute('get_BusyState');
        end
        
        % robot move to placing position
        %P3='(150,150,230,-180,0,180,5)';
        P3='(400,300,300,-180,0,180,5)';
        rob.Move(1,P3);
        %pause
%         load Obj1_01
%         [a b] = size(preArr2);
%         
%         % Trajectory learning
%         for i = 7:a
%             p = [preArr2(i,1) preArr2(i,2) preArr2(i,3) preArr2(i,4) preArr2(i,5) preArr2(i,6) 5];
%     
%             p1 = strhen7(p);
%             rob.Move(1,p1,'NEXT');
%             rob.Execute('Arrive',15);
%             rob.Execute('MotionSkip',[-1,3]);
%         end
%         
%         
        % place the object
        State = caoExt.Execute('get_BusyState');
        while State ~=0
            State = caoExt.Execute('get_BusyState');
        end
        caoExt.Execute('UnChuck',2);
        
         % back to the initial position
        State = caoExt.Execute('get_BusyState');
        while State ~=0
            State = caoExt.Execute('get_BusyState');
        end
        
        P1='(155,0,300,-180,0,180,5)';
        rob.Move(1,P1);

%pause(1)
clear cao % Clear all declear object for next execution
clear ws
clear ctrl
clear rob


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Clear the table
clear;
%close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device
set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)
%% Load R-CNN
load lmFRC_SMObj_05072018_1
net = lmFRCNN;
%% Get image file
frame=getdata(obj,1);

%   im(:,:,:)=frame(86:300,151:500,:);
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
    if (strcmp(label1(j),'RDB')==1)  % Requested object
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
Center = round([bbox3(1,1)+bbox3(1,3)/2,bbox3(1,2)+bbox3(1,4)/2]);
centroid = Center
pause
%% Robot grasping
armset;

% n = length(Center)
% for i = 1:n
    
    % Finding object position of X w.r.t robot
    ax =308.8193;
    bx = -10; % to correct the robot position
    ax1 = 1.3962;
    centroid(2);
    %x = ax1*centroid(i,2)+ax+bx;
    x = ax1*centroid(2)+ax+bx;
    
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
Z = num2str(180);
Rx = num2str(-180);
Ry = num2str(0);
Rz = num2str(180);
fig = num2str(5);

P2 = strcat('(',X,',',Y,',',Z,',',Rx,',',Ry,',',Rz,',',fig,')');
rob.Move(1,P2);

pause


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Clear the table
clear;
%close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device
set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)
%% Load R-CNN
load lmFRC_SMObj_05072018_1
net = lmFRCNN;
%% Get image file
frame=getdata(obj,1);

%   im(:,:,:)=frame(86:300,151:500,:);
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
    if (strcmp(label1(j),'BL')==1)  % Requested object
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
Center = round([bbox3(1,1)+bbox3(1,3)/2,bbox3(1,2)+bbox3(1,4)/2]);
centroid = Center
pause
%% Robot grasping
armset;

% n = length(Center)
% for i = 1:n
    
    % Finding object position of X w.r.t robot
    ax =308.8193;
    bx = -10; % to correct the robot position
    ax1 = 1.3962;
    centroid(2);
    %x = ax1*centroid(i,2)+ax+bx;
    x = ax1*centroid(2)+ax+bx;
    
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
Z = num2str(180);
Rx = num2str(-180);
Ry = num2str(0);
Rz = num2str(180);
fig = num2str(5);

P2 = strcat('(',X,',',Y,',',Z,',',Rx,',',Ry,',',Rz,',',fig,')');
rob.Move(1,P2);

pause


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Clear the table
clear;
%close all;
obj=videoinput('winvideo',1,'YUY2_640x480');  % create video input device
set(obj,'TriggerRepeat',inf) % set graphics object properties
set(obj,'ReturnedColorSpace','rgb')
start(obj)
%% Load R-CNN
load lmFRC_SMObj_05072018_1
net = lmFRCNN;
%% Get image file
frame=getdata(obj,1);

%   im(:,:,:)=frame(86:300,151:500,:);
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
    if (strcmp(label1(j),'BT')==1)  % Requested object
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
Center = round([bbox3(1,1)+bbox3(1,3)/2,bbox3(1,2)+bbox3(1,4)/2]);
centroid = Center
pause
%% Robot grasping
armset;

% n = length(Center)
% for i = 1:n
    
    % Finding object position of X w.r.t robot
    ax =308.8193;
    bx = -10; % to correct the robot position
    ax1 = 1.3962;
    centroid(2);
    %x = ax1*centroid(i,2)+ax+bx;
    x = ax1*centroid(2)+ax+bx;
    
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
Z = num2str(180);
Rx = num2str(-180);
Ry = num2str(0);
Rz = num2str(180);
fig = num2str(5);

P2 = strcat('(',X,',',Y,',',Z,',',Rx,',',Ry,',',Rz,',',fig,')');
rob.Move(1,P2);

pause
