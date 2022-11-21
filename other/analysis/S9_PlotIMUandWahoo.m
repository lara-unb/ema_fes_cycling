% 
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Plot the IMU and Wahoo data.
%

l = cell(1,2);  % Plot legend
l{1} = 'SDSS';  % Set this
l{2} = 'SES';  % Set this
RaceWarmup = 'Race';  % Race or Warmup

%% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   error('Error. Two or more files needed.');
end
TheData = struct();
FileNames = cellfun(@(x) x(1:end-4),Files,'UniformOutput',false);

Prefix = '';
for w = 1:length(Files)
    str = char(FileNames(w));
    Prefix = strcat(Prefix,str(3:12),'_');
end
Prefix = [Prefix,RaceWarmup,'_'];  % Set this

%% Import files into struct
for w = 1:length(Files)
    fprintf('\n\nImporting "%s" mat file...\n',Files{w});
    TheData.(['Sequence',num2str(w)]) = load(Files{w});
end

%% Plot intensity comparison
Fig1 = figure;
colors = lines(7);
p = line();
% l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    PulseWidthTime = D.StimPulseWidthRaw.Time(...
        (D.StimPulseWidthRaw.Time>=D.TimeStimStart+D.TimeOffset) &...
        (D.StimPulseWidthRaw.Time < D.EndTime+D.TimeStimStart+D.TimeOffset));
    PulseWidthData = D.StimPulseWidthRaw.ch1(...
        (D.StimPulseWidthRaw.Time>=D.TimeStimStart+D.TimeOffset) &...
        (D.StimPulseWidthRaw.Time < D.EndTime+D.TimeStimStart+D.TimeOffset));
    if mod(w,2)
        p(w) = plot(PulseWidthTime-PulseWidthTime(1),PulseWidthData,...
            '-','Color',colors(w,:)); hold on
    else
        p(w) = plot(PulseWidthTime-PulseWidthTime(1),PulseWidthData,...
            '--','Color',colors(w,:)); hold on
    end
    temp = plot(D.StartNoAssistance,D.StimPulseWidthRaw.ch1(...
        find(D.StimPulseWidthRaw.Time > D.StartNoAssistance+D.TimeStimStart+D.TimeOffset,1)),'o',...
        'MarkerSize',6,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
    temp = plot([D.StartNoAssistance D.StartNoAssistance],ylim,'--','Color',[colors(w,:),0.5]);
end
hold off
ylabel(['Largura de Pulso (',char(181),'s)'])
xlabel('Tempo (s)')
xlim([0 D.EndTime+15])
legend(p,l,'Interpreter','none','Location','NorthOutside','Orientation','horizontal')

%% Plot speed comparison
disp('Speed comparison (maxNA, meanNA):')
Fig2 = figure;
colors = lines(7);
p = line();
MaxSpeedNA = zeros(1,2);
MeanSpeedNA = zeros(1,2);
% l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    SpeedTime = D.WahooDataSync.secs(D.WahooDataSync.secs <= D.EndTime);
    SpeedData = D.WahooDataSync.kph(D.WahooDataSync.secs <= D.EndTime);
    SpeedNA = D.WahooDataSync.kph((D.WahooDataSync.secs > D.StartNoAssistance) &...
        (D.WahooDataSync.secs <= D.EndTime));
    p(w) = plot(SpeedTime,SpeedData,'Color',colors(w,:)); hold on
    temp = plot(D.StartNoAssistance,D.WahooDataSync.kph(...
        find(D.WahooDataSync.secs == D.StartNoAssistance,1)),'o',...
        'MarkerSize',6,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
%     l{w} = D.Filename(1:end-4);
    MaxSpeedNA(w) = max(SpeedNA);
    MeanSpeedNA(w) = mean(SpeedNA);
    disp(FileNames(w))
    disp(MaxSpeedNA(w))
    disp(MeanSpeedNA(w))
    disp(std(SpeedNA))
end

for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    plot([D.StartNoAssistance D.StartNoAssistance],ylim,'--','Color',[colors(w,:),0.5])
end
hold off
ylabel('Velocidade (km/h)')
xlabel('Tempo (s)')
xlim([0 D.EndTime+15])
legend(p,l,'Interpreter','none','Location','NorthOutside','Orientation','horizontal')

%% Plot distance comparison
disp('Distance comparison (max, maxNA):')
Fig3 = figure;
colors = lines(7);
p = line();
MaxDistance = zeros(1,2);
MaxDistanceNA = zeros(1,2);
% l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    DistanceTime = D.WahooDataSync.secs(D.WahooDataSync.secs <= D.EndTime);
    DistanceData = D.WahooDataSync.km(D.WahooDataSync.secs <= D.EndTime);
    DistanceNA = D.WahooDataSync.km((D.WahooDataSync.secs > D.StartNoAssistance) &...
        (D.WahooDataSync.secs <= D.EndTime));
    p(w) = plot(DistanceTime,DistanceData,'Color',colors(w,:)); hold on
    temp = plot(D.StartNoAssistance,D.WahooDataSync.km(...
        find(D.WahooDataSync.secs == D.StartNoAssistance,1)),'o',...
        'MarkerSize',6,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
%     l{w} = D.Filename(1:end-4);
    MaxDistance(w) = max(DistanceData);
    MaxDistanceNA(w) = max(DistanceNA)-DistanceNA(1);
    disp(FileNames(w))
    disp(MaxDistance(w))
    disp(MaxDistanceNA(w))
end

for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    plot([D.StartNoAssistance D.StartNoAssistance],ylim,'--','Color',[colors(w,:),0.5])
end
hold off
ylabel(['Dist',char(226),'ncia (km)'])
xlabel('Tempo (s)')
xlim([0 D.EndTime+15])
legend(p,l,'Interpreter','none','Location','NorthOutside','Orientation','horizontal')

%% Save figures
saveas(Fig1,[Prefix,'Pulse_Width'],'fig');  % savefig() was giving wrong filenames
saveas(Fig2,[Prefix,'Speed'],'fig');  % savefig() was giving wrong filenames
saveas(Fig3,[Prefix,'Distance'],'fig');  % savefig() was giving wrong filenames

%% Save stats
fid = fopen([Prefix,'Stats.txt'],'wt');

fprintf(fid,'Speed comparison (maxNA, meanNA):\n');
for w = 1:length(Files)
    fprintf(fid,'%s\n', char(FileNames(w)));
    fprintf(fid,'%.6f\n', MaxSpeedNA(w));
    fprintf(fid,'%.6f\n', MeanSpeedNA(w));
end
fprintf(fid,'\nDistance comparison (max, maxNA):\n');
for w = 1:length(Files)
    fprintf(fid,'%s\n', char(FileNames(w)));
    fprintf(fid,'%.6f\n', MaxDistance(w));
    fprintf(fid,'%.6f\n', MaxDistanceNA(w));
end
fclose(fid);
