% 
% MATLAB code - EMA Matrix Experiments
% 2021-01-23
% Lucas de Macedo Pinheiro
% 
%   Convert bagfiles to mat with specific topics.
%

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

%%
for w = 1:length(Files)
    %% Import bag file
    Filename = Files{w};
    fprintf('\n\nImporting "%s" mat file...\n',Filename);
    load(Filename);
    Filename = Files{w}(1:end-4);

    %% Plot
    % Time offset to start with zero - uncomment these for MTRX
%     Toffset = min([CadenceRaw.Time(1),DistanceRaw.Time(1),PedalAngleRaw.Time(1),SpeedRaw.Time(1),...
%         StimCurrentRaw.Time(1),StimMatrixRaw.Time(1),StimPulseWidthRaw.Time(1)]);

    % Time offset to start with zero - uncomment these for CONV
    Toffset = min([CadenceRaw.Time(1),DistanceRaw.Time(1),PedalAngleRaw.Time(1),SpeedRaw.Time(1),...
        StimCurrentRaw.Time(1),StimPulseWidthRaw.Time(1)]);

    figure
    subplot(4,1,1)
    plot(PedalAngleRaw.Time-Toffset,PedalAngleRaw.Data)
    ylabel('Angle (deg)')
    title(Filename(2:end), 'Interpreter', 'none')
    subplot(4,1,2)
    yyaxis left
    plot(CadenceRaw.Time-Toffset,CadenceRaw.Data); hold on
    ylabel('Cadence (km/h)')
    yyaxis right
    plot(SpeedRaw.Time-Toffset,SpeedRaw.Data); hold off
    ylabel('Speed (deg/s)')
    subplot(4,1,3)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch1); hold on
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch2)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch3)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch4)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch5)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch6)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch7)
    plot(StimPulseWidthRaw.Time-Toffset,StimPulseWidthRaw.ch8); hold off
    legend('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8','Location','north',...
        'Orientation','horizontal')
    ylabel('Largura de Pulso (?s)')

    % Matrix Activation - uncomment these for MTRX
%     subplot(4,1,4)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch1); hold on
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch2)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch3)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch4)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch5)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch6)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch7)
%     plot(StimMatrixRaw.Time-Toffset,StimMatrixRaw.ch8); hold off
%     legend('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8','Location','north',...
%         'Orientation','horizontal')
%     ylabel('Current (mA)')
%     xlabel('Time (s)')

    % Stim Current - uncomment these for CONV
    subplot(4,1,4)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch1); hold on
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch2)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch3)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch4)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch5)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch6)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch7)
    plot(StimCurrentRaw.Time-Toffset,StimCurrentRaw.ch8); hold off
    legend('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8','Location','north',...
        'Orientation','horizontal')
    ylabel('Current (mA)')
    xlabel('Time (s)')

    %% Save figure
    savefig(Filename);

end
