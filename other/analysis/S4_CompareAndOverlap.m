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
figure;
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    SpeedTime = D.SpeedRaw.Time(D.SpeedRaw.Time>=D.TimeStimStart);
    SpeedData = D.SpeedRaw.Data(D.SpeedRaw.Time>=D.TimeStimStart);
    plot(SpeedTime-SpeedTime(1),SpeedData); hold on
end
hold off
ylabel('Velocidade (graus/s)')
xlabel('Tempo (s)')
title('Comparacao de Velocidade')
legend(FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

%% Plot cadence comparison
figure;
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    CadenceTime = D.CadenceRaw.Time(D.CadenceRaw.Time>=D.TimeStimStart);
    CadenceData = D.CadenceRaw.Data(D.CadenceRaw.Time>=D.TimeStimStart);
    plot(CadenceTime-CadenceTime(1),CadenceData); hold on
    disp(D.Filename.Filename1)
    disp(mean(CadenceData))
end
hold off
ylabel('Cadencia (km/h)')
xlabel('Tempo (s)')
title('Comparacao de Cadencia')
legend(FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')
disp('')

%% Plot pulse width comparison
figure;
for w = 1:length(Files)
    D = TheData.(['Sequence' num2str(w)]);
    PulseWidthTime = D.StimPulseWidthRaw.Time(D.StimPulseWidthRaw.Time>=D.TimeStimStart);
    PulseWidthData = D.StimPulseWidthRaw.ch1(D.StimPulseWidthRaw.Time>=D.TimeStimStart);
    plot(PulseWidthTime-PulseWidthTime(1),PulseWidthData); hold on
end
hold off
ylabel(['Dist',char(226),'ncia (km)'])
xlabel('Tempo (s)')
title('Comparacao de Largura de Pulso')
legend(FileNames,'Interpreter','none',...
    'Location','southoutside','Orientation','horizontal')

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

