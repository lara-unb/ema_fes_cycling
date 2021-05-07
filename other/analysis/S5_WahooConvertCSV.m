%
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Import CSV files from wahoo and get the speed.
%

% Get from first item of 'Sem auxilio' file
StartNoAssistance = 30;

% Open window for file selection
disp('Select the CSV files...');
File = uigetfile('*.csv','Select The CSV Files','MultiSelect','off');
File = {File};

%% Import file
Filename = File{1};
fprintf('\n\nImporting "%s" CSV file...\n',Filename);
RawCSV = readtable(Filename);
fprintf('StartNoAssistance = %d\n',StartNoAssistance);

%% Check first row for inconsistency
if (RawCSV.secs(2)-RawCSV.secs(1)) ~= 1
    RawCSV(1,:) = [];
end

%% Separate relevant data
disp('Extracting time, km and kph data from CSV file...');
WahooData = RawCSV(:,{'secs','km','kph'});

%% Adjust time
TimeOffset = WahooData.secs(1);
WahooData.secs = WahooData.secs-TimeOffset;
if ~strfind(Filename,'Largada')
    StartNoAssistance = StartNoAssistance-TimeOffset;
end
%% Save matrix data to file
disp('Saving mat file with data...');
save(Filename(1:end-4),'Filename','WahooData','StartNoAssistance');

%% Plot

Fig = figure;
p1 = plot(WahooData.secs,WahooData.kph,'Color',lines(1)); hold on
p2 = plot(StartNoAssistance,WahooData.kph(StartNoAssistance),'o',...
    'MarkerSize',8,'Color',lines(1),'MarkerFaceColor',lines(1)); hold off
xlabel('Tempo (s)')
ylabel('Velocidade (km/h)')
title(Filename(1:end-4),'Interpreter','none')
legend(p1,{Filename(1:end-4)},'Interpreter','none')
