% 
% MATLAB code - EMA Matrix Experiments
% 2021-03-30
% Lucas de Macedo Pinheiro
% 
%   Compare the intensity and speed from different sequences and plot the
% findings starting when stimulation begins.
%

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   error('Error. Two or more files needed.');
end
TheData = struct();
FileNames = cellfun(@(x) x(1:end-4),Files,'UniformOutput',false);
l = cell(1,4);  % Plot legend
l{1} = 'SES 1';  % Set this
l{2} = 'SDSS 1';  % Set this
l{3} = 'SES 2';  % Set this
l{4} = 'SDSS 2';  % Set this

%% Import files into struct
for w = 1:length(Files)
    fprintf('\n\nImporting "%s" mat file...\n',Files{w});
    TheData.(['Sequence',num2str(w)]) = load(Files{w});
end

%% Find when stimulation begins
for w = 1:length(Files)
    PW = TheData.(['Sequence',num2str(w)]).StimPulseWidthRaw;
    % Create new struct element to store the time
    TheData.(['Sequence',num2str(w)]).TimeStimStart = PW.Time(find(PW.ch1>0,1));
end

%% Plot speed comparison
% figure;
% for w = 1:length(Files)
%     D = TheData.(['Sequence' num2str(w)]);
%     SpeedTime = D.SpeedRaw.Time(D.SpeedRaw.Time>=D.TimeStimStart);
%     SpeedData = D.SpeedRaw.Data(D.SpeedRaw.Time>=D.TimeStimStart);
%     plot(SpeedTime-SpeedTime(1),SpeedData); hold on
% end
% hold off
% ylabel('Velocidade (graus/s)')
% xlabel('Tempo (s)')
% title('Comparacao de Velocidade')
% legend(FileNames,'Interpreter','none',...
%     'Location','southoutside','Orientation','horizontal')

%% Plot cadence comparison
figure;
color = [0, 0.4470, 0.7410;
    0.8500, 0.3250, 0.0980;
    0.9290, 0.6940, 0.1250;
    0.4940, 0.1840, 0.5560];

for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    CadenceTime = D.CadenceRaw.Time(D.CadenceRaw.Time>=D.TimeStimStart);
    CadenceData = D.CadenceRaw.Data(D.CadenceRaw.Time>=D.TimeStimStart);
    p(w) = plot(CadenceTime-CadenceTime(1),CadenceData, 'Color', color(w,:)); hold on
    disp(D.Filename.Filename1)
    disp(mean(CadenceData))
    disp(std(CadenceData))
    
    x = CadenceTime-CadenceTime(1);
    curve1 = CadenceData;
    curve1_bottom = curve1 - std(curve1);
    curve1_top = curve1 + std(curve1);
    % plot(x, curve1_bottom, 'r', 'LineWidth', 2);
    % plot(x, curve1_top, 'b', 'LineWidth', 2);
    x2 = [x; flip(x)];
    inBetween = [curve1_top; flip(curve1_bottom)];
    fill(x2, inBetween, color(w,:),'FaceAlpha', 0.2, 'LineStyle', 'none');
end

hold off
ylabel('Cadencia (km/h)')
xlabel('Tempo (s)')
title('Comparacao de Cadencia')
legend(p,l,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')
disp('')

%% Plot pulse width comparison
% figure;
% for w = 1:length(Files)
%     D = TheData.(['Sequence' num2str(w)]);
%     PulseWidthTime = D.StimPulseWidthRaw.Time(D.StimPulseWidthRaw.Time>=D.TimeStimStart);
%     PulseWidthData = D.StimPulseWidthRaw.ch1(D.StimPulseWidthRaw.Time>=D.TimeStimStart);
%     plot(PulseWidthTime-PulseWidthTime(1),PulseWidthData); hold on
% end
% hold off
% ylabel(['Dist',char(226),'ncia (km)'])
% xlabel('Tempo (s)')
% title('Comparacao de Largura de Pulso')
% legend(FileNames,'Interpreter','none',...
%     'Location','southoutside','Orientation','horizontal')

%% Plot cadence and pulse width ratio comparison
% figure;
% for w = 1:length(Files)
%     D = TheData.(['Sequence' num2str(w)]);
%     CadenceTime = D.CadenceRaw.Time(D.CadenceRaw.Time>=D.TimeStimStart);
%     CadenceData = D.CadenceRaw.Data(D.CadenceRaw.Time>=D.TimeStimStart);
%     PulseWidthTime = D.StimPulseWidthRaw.Time(D.StimPulseWidthRaw.Time>=D.TimeStimStart);
%     PulseWidthData = D.StimPulseWidthRaw.ch1(D.StimPulseWidthRaw.Time>=D.TimeStimStart);
%     % Decide which time to use based on fewer samples
%     if length(PulseWidthTime) < length(CadenceTime)
%         Samples = length(PulseWidthTime);
%         Time = PulseWidthTime;
%     else
%         Samples = length(CadenceTime);
%         Time = CadenceTime;
%     end
%     % Compute the ratio between cadence and pulse width (microseconds)
%     Ratio = CadenceData(1:Samples)./double(PulseWidthData(1:Samples));
%     plot(Time-Time(1),Ratio); hold on
% end
% hold off
% ylabel(['Cadencia(km/h)/Largura de Pulso(',char(181),'s)'])
% xlabel('Tempo Deslocado (s)')
% title('Comparacao da Razao entre Cadencia e Largura de Pulso')
% legend(FileNames,'Interpreter','none',...
%     'Location','southoutside','Orientation','horizontal')

