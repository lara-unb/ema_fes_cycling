%
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Import CSV or XLS files from wahoo and get the speed.
%

% Open window for file selection
disp('Select the files...');
File = uigetfile('*','Select The Files','MultiSelect','off');
File = {File};

%% Import file
Filename = File{1};
fprintf('\n\nImporting "%s" file...\n',Filename);
RawData = readtable(Filename,'ReadVariableNames',true);

% Time in seconds when assistence was over
StartNoAssistance = 0;
% StartNoAssistance = RawData{2,4}{1}(1:3);
% StartNoAssistance = str2num(StartNoAssistance);
% fprintf('StartNoAssistance = %d\n',StartNoAssistance);

%% Check first row for inconsistency
if (RawData.secs(2)-RawData.secs(1)) ~= 1
    RawData(1,:) = [];
end

%% Separate relevant data
disp('Extracting time, km and kph data from file...');
WahooData = RawData(:,{'secs','km','kph'});

%% Adjust time
% TimeOffset = WahooData.secs(1);
% WahooData.secs = WahooData.secs-TimeOffset;
% if ~strfind(Filename,'Largada')
%     StartNoAssistance = StartNoAssistance-TimeOffset;
% end
%% Save matrix data to file
disp('Saving mat file with data...');
save([Filename(1:end-4),'mat'],'Filename','WahooData','StartNoAssistance');

%% Plot

Fig = figure;
p1 = plot(WahooData.secs,WahooData.kph,'Color',lines(1)); hold on
p2 = plot(StartNoAssistance,WahooData.kph(StartNoAssistance+1),'o',...
    'MarkerSize',8,'Color',lines(1),'MarkerFaceColor',lines(1)); hold off
xlabel('Time (s)')
ylabel('Speed (km/h)')
title(Filename(1:end-4),'Interpreter','none')
% legend(p1,{Filename(1:end-4)},'Interpreter','none')
