function varargout = BlobWarper3000(varargin)
% BLOBWARPER3000 MATLAB code for BlobWarper3000.fig
%      BLOBWARPER3000, by itself, creates a new BLOBWARPER3000 or raises the existing
%      singleton*.
%
%      H = BLOBWARPER3000 returns the handle to a new BLOBWARPER3000 or the handle to
%      the existing singleton*.
%
%      BLOBWARPER3000('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BLOBWARPER3000.M with the given input arguments.
%
%      BLOBWARPER3000('Property','Value',...) creates a new BLOBWARPER3000 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BlobWarper3000_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BlobWarper3000_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BlobWarper3000

% Last Modified by GUIDE v2.5 28-Nov-2024 16:25:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BlobWarper3000_OpeningFcn, ...
                   'gui_OutputFcn',  @BlobWarper3000_OutputFcn, ...
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


% --- Executes just before BlobWarper3000 is made visible.
function BlobWarper3000_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BlobWarper3000 (see VARARGIN)

% Choose default command line output for BlobWarper3000
handles.output = hObject;

% Display welcome image
if isfile('welcome.png')
    welcomeIMG = imread('welcome.png');
    axes(handles.ax_main)
    imshow(welcomeIMG);
    clear welcomeIMG
end% if say hello

% Listen to sliders to immediately update display
addlistener(handles.sl_frame,...
    'Value','PreSet',...
    @(~,~)set(handles.ed_current_frame, 'String', num2str(round(get(handles.sl_frame, 'Value'), 0))));
addlistener(handles.sl_pitch,...
    'Value','PreSet',...
    @(~,~)set(handles.ed_pitch_angle, 'String', num2str(round(get(handles.sl_pitch, 'Value'), 0))));
addlistener(handles.sl_roll,...
    'Value','PreSet',...
    @(~,~)set(handles.ed_roll_angle, 'String', num2str(round(get(handles.sl_roll, 'Value'), 0))));
addlistener(handles.sl_yaw,...
    'Value','PreSet',...
    @(~,~)set(handles.ed_yaw_angle, 'String', num2str(round(get(handles.sl_yaw, 'Value'), 0))));

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes BlobWarper3000 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = BlobWarper3000_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Update figure
function handles = updateFigure(handles)

% Update info which frame is displayed
set(handles.sl_frame, 'Value', handles.CurrFrame)
set(handles.ed_current_frame, 'String', num2str(handles.CurrFrame))

% Update current angles
% --- pitch
set(handles.sl_pitch, 'Value', handles.pitch_angle)
set(handles.ed_pitch_angle, 'String', num2str(handles.pitch_angle))
% --- roll
set(handles.sl_roll, 'Value', handles.roll_angle)
set(handles.ed_roll_angle, 'String', num2str(handles.roll_angle))
% --- yaw
set(handles.sl_yaw, 'Value', handles.yaw_angle)
set(handles.ed_yaw_angle, 'String', num2str(handles.yaw_angle))

% Get the homography matrix
handles.homography_matrix = get_homo(handles.Video.Width, handles.Video.Height, handles.pitch_angle, handles.yaw_angle, handles.roll_angle);

% Prepare frame (thresholding etc)
[frame, handles.spat_ref] = prepareFrame(handles);

% Display frame
axes(handles.ax_main); cla
imagesc(frame);
axis(handles.ax_main, 'equal', 'tight', 'off')
drawnow;



function [frame, spat_ref] = prepareFrame(handles)
% Read frame
frame = read(handles.Video.Obj, handles.CurrFrame);
% 2-D projective geometric transformation using postmultiply convention
tform = projective2d(handles.homography_matrix');
% Apply the projective transformation
[frame, spat_ref] = imwarp(frame, tform);


function homography_matrix = get_homo(width, height, pitch_angle, yaw_angle, roll_angle)
% Estimate the focal length
focal_length = width;
% Principal point (center of the image)
cx = width / 2;
cy = height / 2;
% Construct the camera intrinsic matrix K
K = [focal_length,       0,    cx;
             0,  focal_length,  cy;
             0,       0,        1];
% Rotation around x-axis (pitch)
Rx = [1,               0,                0;
      0,  cosd(pitch_angle), -sind(pitch_angle);
      0,  sind(pitch_angle),  cosd(pitch_angle)];
% Rotation around y-axis (yaw)
Ry = [cosd(yaw_angle),   0,  sind(yaw_angle);
                0,      1,       0;
     -sind(yaw_angle),   0,  cosd(yaw_angle)];
% Rotation around z-axis (roll)
Rz = [cosd(roll_angle), -sind(roll_angle), 0;
      sind(roll_angle),  cosd(roll_angle), 0;
               0,               0,       1];
% Combined rotation matrix
R = Rz * Ry * Rx;
% Compute the Homography Matrix
homography_matrix = K * R * inv(K);



% --------------------------------------------------------------------
function mn_open_video_Callback(hObject, eventdata, handles)
% hObject    handle to mn_open_video (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Ask the user to specify a video file
[handles.CurrFile, handles.CurrFilePath] = uigetfile({'*.mp4'; '*.avi'}, 'Select a video file');

% Waitbar for inpatient people
hWait = waitbar(0, {'Please wait while we create a video object to'; 'read video data from the file specified...'});

% Extract the basename
handles.CurrTrial = handles.CurrFile(1:end-4);

% Import video object
handles.Video.Obj = VideoReader([handles.CurrFilePath, handles.CurrFile]);
handles.Video.NrFrames = get(handles.Video.Obj, 'numberOfFrames')-1;
handles.Video.Width = get(handles.Video.Obj, 'Width');
handles.Video.Height = get(handles.Video.Obj, 'Height');

% Indicate that something is happening
waitbar(0.5, hWait); pause(0.1)

% Prepare transformation
handles.focal_length = handles.Video.Width;
handles.pitch_angle = 0;
handles.yaw_angle = 0;
handles.roll_angle = 0;

% Get the homography matrix
handles.homography_matrix = get_homo(handles.Video.Width, handles.Video.Height, handles.pitch_angle, handles.yaw_angle, handles.roll_angle);

% Update display of current file
set(handles.tx_title, 'String', handles.CurrTrial)

% Update frame slider
set(handles.sl_frame,...
    'Min', 1,...
    'Max', handles.Video.NrFrames,...
    'SliderStep', [1/(handles.Video.NrFrames-1) , 10/(handles.Video.NrFrames-1)])

% Update pitch_angle slider
set(handles.sl_pitch,...
    'Min', -floor(atan2d(handles.Video.Width, handles.Video.Height)),...
    'Max', floor(atan2d(handles.Video.Width, handles.Video.Height)),...
    'SliderStep', [1/(2*floor(atan2d(handles.Video.Width, handles.Video.Height))) , 10/(2*floor(atan2d(handles.Video.Width, handles.Video.Height)))])

% Update roll_angle slider
set(handles.sl_roll,...
     'Min', -180,...
    'Max', 180,...
    'SliderStep', [1/360 , 10/360])

% Update yaw_angle slider
set(handles.sl_yaw,...
     'Min', -45,...
    'Max', 45,...
    'SliderStep', [1/90 , 10/90])

% Update figure
handles.CurrFrame = 1;
handles = updateFigure(handles);

% Waitbar
waitbar(1, hWait); pause(0.1)
close(hWait)

% Update handles structure
guidata(hObject, handles);


% --------------------------------------------------------------------
function mn_save_trans_Callback(hObject, eventdata, handles)
% hObject    handle to mn_save_trans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Waitbar for inpatient people
hWait = waitbar(0, 'Please wait while we save everything');

% Get all settings
WarpSettings.pitch = handles.pitch_angle;
WarpSettings.roll = handles.roll_angle;
WarpSettings.yaw = handles.yaw_angle;
WarpSettings.focal_length = handles.focal_length;
WarpSettings.homography_matrix = handles.homography_matrix;
WarpSettings.spat_ref = handles.spat_ref;
WarpSettings.fcn_warp = @(img, homography_matrix) imwarp(img, projective2d(homography_matrix'));
WarpSettings.fcn_trans_coord = @(x, y, homography_matrix, spat_ref) ...
    (transformPointsForward(projective2d(homography_matrix'), [x, y]) - [spat_ref.XWorldLimits(1), spat_ref.YWorldLimits(1)]);

% Waitbar
waitbar(1,hWait)
close(hWait)

% Save
save([handles.CurrFilePath, handles.CurrTrial, '_warp.mat'], 'WarpSettings')



function ed_current_frame_Callback(hObject, eventdata, handles)
% Update figure
handles.CurrFrame = str2double(get(hObject, 'String'));
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ed_current_frame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_current_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_pitch_angle_Callback(hObject, eventdata, handles)
% Update figure
handles.pitch_angle = str2double(get(hObject, 'String'));
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ed_pitch_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_pitch_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_roll_angle_Callback(hObject, eventdata, handles)
% Update figure
handles.roll_angle = str2double(get(hObject, 'String'));
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ed_roll_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_roll_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_yaw_angle_Callback(hObject, eventdata, handles)
% Update figure
handles.yaw_angle = str2double(get(hObject, 'String'));
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function ed_yaw_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_yaw_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function sl_frame_Callback(hObject, eventdata, handles)
% Update figure
handles.CurrFrame = round(get(hObject, 'Value'));
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function sl_frame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sl_pitch_Callback(hObject, eventdata, handles)
% Update figure
handles.pitch_angle = get(hObject, 'Value');
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function sl_pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sl_roll_Callback(hObject, eventdata, handles)
% Update figure
handles.roll_angle = get(hObject, 'Value');
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function sl_roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sl_yaw_Callback(hObject, eventdata, handles)
% Update figure
handles.yaw_angle = get(hObject, 'Value');
handles = updateFigure(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function sl_yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
