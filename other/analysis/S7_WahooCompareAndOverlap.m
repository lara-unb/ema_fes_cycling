% 
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Compare the distance and speed from different sequences and plot the
% race findings.
%

% EndTime = 900;  % Use to chop off data at the end

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   error('Error. Two or more files needed.');
end
TheData = struct();

% Get race or warmup from file name
if isempty(strfind(Files(1),'Corrida'))
    Type = 'Race';
else
    Type = 'Warmup';
end

% FileNames = cellfun(@(x) x(1:end-4),Files,'UniformOutput',false);
FileNames = cellfun(@(x) regexp(x,'^([^_]*_[^_]*)','once','tokens'),Files);

%% Import files into struct
for w = 1:length(Files)
    fprintf('\n\nImporting "%s" mat file...\n',Files{w});
    TheData.(['Sequence',num2str(w)]) = load(Files{w});
end

%% Plot speed comparison
disp([Type,' - Speed comparison (maxNA, meanNA):'])
Fig = figure;
colors = lines(7);
p = line();
l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
%     if w==2
%         SpeedTime = D.WahooData.secs((D.WahooData.secs > 20) & (D.WahooData.secs <= EndTime));
%         SpeedData = D.WahooData.kph(20+2:EndTime+1);
%     else
%         SpeedTime = D.WahooData.secs(D.WahooData.secs <= EndTime);
%         SpeedData = D.WahooData.kph(1:EndTime+1);
%     end
%     SpeedNA = D.WahooData.kph(D.StartNoAssistance+2:EndTime+1);
    SpeedTime = D.WahooData.secs;
    SpeedData = D.WahooData.kph;
    SpeedNA = D.WahooData.kph;
    p(w) = plot(SpeedTime,SpeedData,'Color',colors(w,:)); hold on
%     temp = plot(D.StartNoAssistance,D.WahooData.kph(D.StartNoAssistance+1),'o',...
%         'MarkerSize',8,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
%     l{w} = D.Filename(1:end-4);
    disp(FileNames(w))
    disp(max(SpeedNA))
    disp(mean(SpeedNA))
end
hold off
ylabel('Speed (km/h)')
xlabel('Time (s)')
title(Type)
legend(p,FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

%% Save figure
savefig(Fig,[Type,'_Speed']);

%% Plot distance comparison
disp([Type,' - Distance comparison (max, maxNA):'])
Fig = figure;
colors = lines(7);
p = line();
l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
%     if w==2
%         DistanceTime = D.WahooData.secs((D.WahooData.secs > 20) & (D.WahooData.secs <= EndTime));
%         DistanceData = D.WahooData.km(20+2:EndTime+1);
%     else
%         DistanceTime = D.WahooData.secs(D.WahooData.secs <= EndTime);
%         DistanceData = D.WahooData.km(1:EndTime+1);
%     end
    DistanceTime = D.WahooData.secs;
    DistanceData = D.WahooData.km;
    DistanceNA = DistanceData-DistanceData(1);
    p(w) = plot(DistanceTime,DistanceData,'Color',colors(w,:)); hold on
%     temp = plot(D.StartNoAssistance,D.WahooData.km(D.StartNoAssistance+1),'o',...
%         'MarkerSize',8,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
%     l{w} = D.Filename(1:end-4);
    disp(FileNames(w))
    disp(max(DistanceData))
    disp(max(DistanceNA))
end
hold off
ylabel('Distance (km)')
xlabel('Time (s)')
title(Type)
legend(p,FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

%% Save figure
savefig(Fig,[Type,'_Distance']);
