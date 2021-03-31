% 
% MATLAB code - EMA Matrix Experiments
% 2021-03-29
% Lucas de Macedo Pinheiro
% 
%   Plot the main topics in a figure with subplots.
%

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

%%
for w = 1:length(Files)
    %% Import file
    CurrentFileName = Files{w};
    fprintf('\n\nImporting "%s" mat file...\n',CurrentFileName);
    D = load(CurrentFileName);
    CurrentFileName = CurrentFileName(1:end-4);

    %% Plot
    try  % Check if D.StimMatrixRaw is present, if not it is not a matrix dataset
        % Time offset to start with zero
        TimeOffset = min([D.CadenceRaw.Time(1),D.DistanceRaw.Time(1),...
            D.PedalAngleRaw.Time(1),D.SpeedRaw.Time(1),D.StimCurrentRaw.Time(1),...
            D.StimMatrixRaw.Time(1),D.StimPulseWidthRaw.Time(1)]);
    catch e  % Carry on considering a conventional dataset
        if strcmp(e.identifier,'MATLAB:nonExistentField')
            % Time offset to start with zero
            TimeOffset = min([D.CadenceRaw.Time(1),D.DistanceRaw.Time(1),...
                D.PedalAngleRaw.Time(1),D.SpeedRaw.Time(1),D.StimCurrentRaw.Time(1),...
                D.StimPulseWidthRaw.Time(1)]);
        else  % Pass on the error
            rethrow(e)
        end
    end

    Fig = figure;
    subplot(4,1,1)
    plot(D.PedalAngleRaw.Time-TimeOffset,D.PedalAngleRaw.Data)
    ylabel('Angulo (graus)')
    title(CurrentFileName(2:end), 'Interpreter', 'none')
    subplot(4,1,2)
    yyaxis left
    plot(D.CadenceRaw.Time-TimeOffset,D.CadenceRaw.Data); hold on
    ylabel('Cadencia (km/h)')
    yyaxis right
    plot(D.SpeedRaw.Time-TimeOffset,D.SpeedRaw.Data); hold off
    ylabel('Velocidade (graus/s)')
    subplot(4,1,3)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch1); hold on
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch2)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch3)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch4)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch5)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch6)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch7)
    plot(D.StimPulseWidthRaw.Time-TimeOffset,D.StimPulseWidthRaw.ch8); hold off
    legend('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8','Location','north',...
        'Orientation','horizontal')
    ylabel(['Largura de Pulso (',char(181),'s)'])

    try  % Check if D.StimMatrixRaw is present, if not it is not a matrix dataset
        % Matrix Activation
        subplot(4,1,4)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch1); hold on
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch2)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch3)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch4)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch5)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch6)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch7)
        plot(D.StimMatrixRaw.Time-TimeOffset,D.StimMatrixRaw.ch8); hold off
        legend('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8',...
            'Location','north','Orientation','horizontal')
        ylabel('Corrente (mA)')
        xlabel('Tempo (s)')
    catch e  % Carry on considering a conventional dataset
        if strcmp(e.identifier,'MATLAB:nonExistentField')
            % Plain Stim Current
            subplot(4,1,4)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch1); hold on
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch2)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch3)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch4)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch5)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch6)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch7)
            plot(D.StimCurrentRaw.Time-TimeOffset,D.StimCurrentRaw.ch8); hold off
            legend('ch1','ch2','ch3','ch4','ch5','ch6','ch7','ch8',...
                'Location','north','Orientation','horizontal')
            ylabel('Corrente (mA)')
            xlabel('Tempo (s)')
        else  % Pass on the error
            rethrow(e)
        end
    end

    %% Save figure
    saveas(Fig, CurrentFileName, 'fig');  % savefig() was giving wrong filenames

end
