%
% MATLAB code - EMA Matrix Experiments
% 2021-01-23
% Lucas de Macedo Pinheiro
% 
%   Convert bagfiles to mat with specific topics.
%

% Open window for file selection
disp('Select the bagfiles...');
Files = uigetfile('*.bag','Select The Bagfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

%%
for w = 1:length(Files)
    %% Import file
    Filename = Files{w};
    fprintf('\n\nImporting "%s" bagfile...\n',Filename);
    RawBag = rosbag(Filename);
    Filename = Files{w}(1:end-4);

    %% Separate matrix activation data - uncomment this section for MTRX
%     disp('Extracting matrix activation data from bagfile...');
%     StimMatrixTopicFromBag = select(RawBag,'Topic','/ema/stimulator/matrix/activation');
%     StimMatrixRaw = cell2table(readMessages(StimMatrixTopicFromBag));
%     StimMatrixRaw = cell2mat({StimMatrixRaw.Var1.Data})';
%     StimMatrixRaw = table(StimMatrixRaw(:,2),StimMatrixRaw(:,3),StimMatrixRaw(:,4),...
%         StimMatrixRaw(:,5),StimMatrixRaw(:,6),StimMatrixRaw(:,7),StimMatrixRaw(:,8),...
%         StimMatrixRaw(:,9),'VariableNames',{'ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8'});
%     StimMatrixRaw.Time = StimMatrixTopicFromBag.MessageList.Time;

    %% Separate stim current data
    disp('Extracting stim current data from bagfile...');
    StimCurrentTopicFromBag = select(RawBag,'Topic','/ema/stimulator/current');
    StimCurrentRaw = cell2table(readMessages(StimCurrentTopicFromBag));
    StimCurrentRaw = cell2mat({StimCurrentRaw.Var1.Data})';
    StimCurrentRaw = table(StimCurrentRaw(:,2),StimCurrentRaw(:,3),StimCurrentRaw(:,4),...
        StimCurrentRaw(:,5),StimCurrentRaw(:,6),StimCurrentRaw(:,7),StimCurrentRaw(:,8),...
        StimCurrentRaw(:,9),'VariableNames',{'ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8'});
    StimCurrentRaw.Time = StimCurrentTopicFromBag.MessageList.Time;

    %% Separate stim pulse width data
    disp('Extracting stim pulse width data from bagfile...');
    StimPulseWidthTopicFromBag = select(RawBag,'Topic','/ema/stimulator/pulse_width');
    StimPulseWidthRaw = cell2table(readMessages(StimPulseWidthTopicFromBag));
    StimPulseWidthRaw = cell2mat({StimPulseWidthRaw.Var1.Data})';
    StimPulseWidthRaw = table(StimPulseWidthRaw(:,2),StimPulseWidthRaw(:,3),StimPulseWidthRaw(:,4),...
        StimPulseWidthRaw(:,5),StimPulseWidthRaw(:,6),StimPulseWidthRaw(:,7),StimPulseWidthRaw(:,8),...
        StimPulseWidthRaw(:,9),'VariableNames',{'ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8'});
    StimPulseWidthRaw.Time = StimPulseWidthTopicFromBag.MessageList.Time;

    %% Separate angle data
    disp('Extracting angle data from bagfile...');
    PedalAngleTopicFromBag = select(RawBag,'Topic','/ema/trike/angle');
    PedalAngleRaw = timeseries(PedalAngleTopicFromBag);

    %% Separate cadence data
    disp('Extracting cadence data from bagfile...');
    CadenceTopicFromBag = select(RawBag,'Topic','/ema/trike/cadence');
    CadenceRaw = timeseries(CadenceTopicFromBag);

    %% Separate distance data
    disp('Extracting distance data from bagfile...');
    DistanceTopicFromBag = select(RawBag,'Topic','/ema/trike/distance');
    DistanceRaw = timeseries(DistanceTopicFromBag);

    %% Separate elapsed time data
    disp('Extracting elapsed time data from bagfile...');
    ElapsedTopicFromBag = select(RawBag,'Topic','/ema/trike/elapsed');
    ElapsedRaw = timeseries(ElapsedTopicFromBag);

    %% Separate speed data
    disp('Extracting speed data from bagfile...');
    SpeedTopicFromBag = select(RawBag,'Topic','/ema/trike/speed');
    SpeedRaw = timeseries(SpeedTopicFromBag);

    %% Separate status data
    % disp('Extracting status data from bagfile...');
    % StatusTopicFromBag = select(RawBag,'Topic','/ema/trike/status');
    % StatusRaw = cell2table(readMessages(StatusTopicFromBag));
    % StatusRaw.Time = StatusTopicFromBag.MessageList.Time;

    %% Save data to file - uncomment this section for MTRX
%     disp('Saving mat file...');
%     save(Filename,'StimPulseWidthRaw','StimMatrixRaw','StimCurrentRaw','SpeedRaw',...
%         'PedalAngleRaw','Filename','DistanceRaw','ElapsedRaw','CadenceRaw');

    %% Save data to file - uncomment this section for CONV
    disp('Saving mat file...');
    save(Filename,'StimPulseWidthRaw','StimCurrentRaw','SpeedRaw',...
        'PedalAngleRaw','Filename','DistanceRaw','ElapsedRaw','CadenceRaw');
end
