% 
% MATLAB code - EMA Matrix Experiments
% 2021-08-12
% Lucas de Macedo Pinheiro
% 
%   Compare the intensity and speed from different sequences and plot the
% findings for Wahoo data.
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
if isempty(regexp(Files{1},'Corrida','once'))
    Type = 'Warmup';
else
    Type = 'Race';
end

% FileNames = cellfun(@(x) x(1:end-4),Files,'UniformOutput',false);
FileNames = cellfun(@(x) regexp(x,'^([^_]*_[^_]*)','once','tokens'),Files);
l = FileNames;  % Plot legend

%% Import files into struct
for w = 1:length(Files)
    fprintf('\n\nImporting "%s" mat file...\n',Files{w});
    TheData.(['Sequence',num2str(w)]) = load(Files{w});
end

%% Plot speed comparison
disp([Type,' - Speed comparison (mean, STD):'])
Fig = figure;
% colors = lines(7);
color = [0, 0.4470, 0.7410;
    0.8500, 0.3250, 0.0980;
    0.9290, 0.6940, 0.1250;
    0.4940, 0.1840, 0.5560];
p = line();
% l = cell(1,length(Files));
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    SpeedTime = D.WahooData.secs;
    SpeedData = D.WahooData.kph;
    p(w) = plot(SpeedTime,SpeedData,'Color',color(w,:)); hold on
    disp(FileNames(w))
    disp(mean(SpeedData))
    disp(std(SpeedData))
    
    x = SpeedTime;
    curve1 = SpeedData;
    curve1_bottom = curve1 - std(curve1);
    curve1_top = curve1 + std(curve1);
    % plot(x, curve1_bottom, 'r', 'LineWidth', 2);
    % plot(x, curve1_top, 'b', 'LineWidth', 2);
    x2 = [x; flip(x)];
    inBetween = [curve1_top; flip(curve1_bottom)];
    fill(x2, inBetween, color(w,:),'FaceAlpha', 0.2, 'LineStyle', 'none');
end

hold off
ylabel('Speed (km/h)')
xlabel('Time (s)')
title(Type)
legend(p,l,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

%% Save figure
savefig(Fig,[Type,'_Speed_STD']);

