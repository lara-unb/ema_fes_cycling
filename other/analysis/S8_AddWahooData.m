% 
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Join the Wahoo data with the data from ROS. 
%

EndTime = 510;  % Set this

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   error('Error. Two or more files needed.');
end

%% Import files into struct
for w = 1:length(Files)
    fprintf('\n\nImporting "%s" mat file...\n',Files{w});
    load(Files{w});
    if isa(Filename,'struct')
        FilenameCopy = Filename;
    else
        clear('Filename');
    end
end
Filename = FilenameCopy;
% StartNoAssistance = 360;  % Set this
TimeStimStart = StimPulseWidthRaw.Time(find(StimPulseWidthRaw.ch1>0,1)) - TimeOffset;

%% Plot
Fig = figure;
p1 = plot(WahooData.secs,WahooData.kph,'Color',lines(1)); hold on
p2 = plot(StartNoAssistance,WahooData.kph(find(WahooData.secs == StartNoAssistance,1)),'o',...
    'MarkerSize',8,'Color',lines(1),'MarkerFaceColor',lines(1));
IMUCadenceTime = CadenceRaw.Time(CadenceRaw.Time>=TimeOffset+TimeStimStart);
IMUCadenceData = CadenceRaw.Data(CadenceRaw.Time>=TimeOffset+TimeStimStart);
p3 = plot(IMUCadenceTime-IMUCadenceTime(1),IMUCadenceData); hold off
xlabel('Tempo (s)')
ylabel('Velocidade (km/h)')
% title(Filename(1:end-4),'Interpreter','none')
legend([p1 p3],{'Wahoo','IMU'},'Interpreter','none')

%% Wahoo vs IMU time adjustment
TimeSyncDiff = 0;  % Set this
WahooDataSync = WahooData;
WahooDataSync.secs = WahooDataSync.secs+TimeSyncDiff;
if TimeSyncDiff < 0
    WahooDataSync = WahooDataSync(WahooDataSync.secs >= 0,:);
end

%% Plot
figure(Fig); hold on
colors = lines(2);
p1 = plot(WahooDataSync.secs,WahooDataSync.kph,'Color',colors(2,:));
p2 = plot(StartNoAssistance,WahooDataSync.kph(find(WahooDataSync.secs == StartNoAssistance,1)),'o',...
    'MarkerSize',8,'Color',colors(2,:),'MarkerFaceColor',colors(2,:)); hold off

%% Save data to file
fprintf('\nEndTime = %d\n',EndTime);
fprintf('StartNoAssistance = %d\n',StartNoAssistance);
fprintf('TimeSyncDiff = %d\n',TimeSyncDiff);
disp('Saving mat file...');
save(['_',Filename.Filename1(1:end-1),'ROSWH'],'StimPulseWidthRaw','StimCurrentRaw','SpeedRaw',...
            'PedalAngleRaw','Filename','DistanceRaw','ElapsedRaw','CadenceRaw','TimeOffset',...
            'TimeStimStart','WahooData','WahooDataSync','EndTime','StartNoAssistance',...
            'TimeSyncDiff');
