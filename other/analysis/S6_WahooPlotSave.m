% 
% MATLAB code - EMA Matrix Experiments
% 2021-05-05
% Lucas de Macedo Pinheiro
% 
%   Plot the Wahoo data in a figure with subplots.
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
    load(CurrentFileName);
    % CurrentFileName = CurrentFileName(1:end-4);

    %% Plot
    Fig = figure;
    subplot(2,1,1)
    p1 = plot(WahooData.secs,WahooData.kph,'Color',lines(1)); hold on
    p2 = plot(StartNoAssistance,WahooData.kph(StartNoAssistance+1),'o',...
        'MarkerSize',8,'Color',lines(1),'MarkerFaceColor',lines(1)); hold off
    xlabel('Time (s)')
    ylabel('Speed (km/h)')
    title(strtok(Filename,'.'),'Interpreter','none')
%     legend(p1,{Filename(1:end-4)},'Interpreter','none')
    
    subplot(2,1,2)
    p1 = plot(WahooData.secs,WahooData.km,'Color',lines(1)); hold on
    p2 = plot(StartNoAssistance,WahooData.km(StartNoAssistance+1),'o',...
        'MarkerSize',8,'Color',lines(1),'MarkerFaceColor',lines(1)); hold off
    xlabel('Time (s)')
    ylabel(['Distance (km)'])
%     title(strtok(Filename,'.'),'Interpreter','none')
%     legend(p1,{Filename(1:end-4)},'Interpreter','none')

    %% Save figure
    saveas(Fig,strtok(Filename,'.'),'fig');  % savefig() was giving wrong filenames

end
