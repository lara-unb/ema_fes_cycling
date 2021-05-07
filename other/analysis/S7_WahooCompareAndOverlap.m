% 
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Compare the distance and speed from different sequences and plot the
% race findings.
%

EndTime = 900;

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   error('Error. Two or more files needed.');
end
TheData = struct();
FileNames = cellfun(@(x) x(1:end-4),Files,'UniformOutput',false);

%% Import files into struct
for w = 1:length(Files)
    fprintf('\n\nImporting "%s" mat file...\n',Files{w});
    TheData.(['Sequence',num2str(w)]) = load(Files{w});
end

%% Plot speed comparison
disp('Speed comparison (maxNA, meanNA):')
Fig = figure;
colors = lines(7);
p = line();
l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    if w==2
        SpeedTime = D.WahooData.secs((D.WahooData.secs > 20) & (D.WahooData.secs <= EndTime));
        SpeedData = D.WahooData.kph(20+2:EndTime+1);
    else
        SpeedTime = D.WahooData.secs(D.WahooData.secs <= EndTime);
        SpeedData = D.WahooData.kph(1:EndTime+1);
    end
    SpeedNA = D.WahooData.kph(D.StartNoAssistance+2:EndTime+1);
    p(w) = plot(SpeedTime,SpeedData,'Color',colors(w,:)); hold on
    temp = plot(D.StartNoAssistance,D.WahooData.kph(D.StartNoAssistance+1),'o',...
        'MarkerSize',8,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
%     l{w} = D.Filename(1:end-4);
    disp(FileNames(w))
    disp(max(SpeedNA))
    disp(mean(SpeedNA))
end
hold off
ylabel('Velocidade (km/h)')
xlabel('Tempo (s)')
title(['Compara',char(231),char(227),'o de Velocidade'])
legend(p,FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

%% Save figure
savefig(Fig,['Compare_Velo_',char(FileNames(1))]);

%% Plot distance comparison
disp('Distance comparison (max, maxNA):')
Fig = figure;
colors = lines(7);
p = line();
l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    if w==2
        DistanceTime = D.WahooData.secs((D.WahooData.secs > 20) & (D.WahooData.secs <= EndTime));
        DistanceData = D.WahooData.km(20+2:EndTime+1);
    else
        DistanceTime = D.WahooData.secs(D.WahooData.secs <= EndTime);
        DistanceData = D.WahooData.km(1:EndTime+1);
    end
    DistanceNA = D.WahooData.km(D.StartNoAssistance+2:EndTime+1);
    DistanceNA = DistanceNA-DistanceNA(1);
    p(w) = plot(DistanceTime,DistanceData,'Color',colors(w,:)); hold on
    temp = plot(D.StartNoAssistance,D.WahooData.km(D.StartNoAssistance+1),'o',...
        'MarkerSize',8,'Color',colors(w,:),'MarkerFaceColor',colors(w,:));
%     l{w} = D.Filename(1:end-4);
    disp(FileNames(w))
    disp(max(DistanceData))
    disp(max(DistanceNA))
end
hold off
ylabel(['Dist',char(226),'ncia (km)'])
xlabel('Tempo (s)')
title(['Compara',char(231),char(227),'o de Dist',char(226),'ncia'])
legend(p,FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

%% Save figure
savefig(Fig,['Compare_Dist_',char(FileNames(1))]);
