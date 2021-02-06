% 
% MATLAB code - EMA Matrix Experiments
% 2021-02-05
% Lucas de Macedo Pinheiro
% 
%   Parse the txt file and check the topic's rate. An example of txt file
% looks like this:
%
% -----------------------------------------------------------
% subscribed to [/ema/stimulator/matrix/activation]
% average rate: 47.989
%       min: 0.020s max: 0.022s std dev: 0.00050s window: 48
% average rate: 48.001
% ...
% -----------------------------------------------------------
%
%   And it comes from the following terminal command during ROS execution:
% rostopic hz -w 48 /ema/stimulator/matrix/activation > ~/file.txt
%

% Declare constants
ExpectedRate = 48;

% Open window for file selection
disp('Select the txt file...');
Files = uigetfile('*.txt','Select The Text File');
Filename = Files;

%% Import the file, parse the data and close the file
FileID = fopen(Filename,'r');
Filename = Filename(1:end-4);
Format = ['average rate:' '%f\n\t' 'min:' '%f' 's max:' '%f' 's std dev:'...
    '%f' 's window:' '%f'];
ParsedData = textscan(FileID,Format,'HeaderLines',1,'CollectOutput',1,...
    'CommentStyle','no new messages');
ParsedData = ParsedData{1};

% Remove rows if rate is less than 2 hz, happens after 'no new messages'
ParsedData(ParsedData(:,1)<2,:) = [];

VarNames = {'AvgRate','MinTime','MaxTime','SD','Window'};
ParsedData = array2table(ParsedData,'VariableNames',VarNames);
fclose(FileID);

%% Plot
ExpectedPeriod = round(1/ExpectedRate,3);

figure
subplot(5,2,[1,2])
plot(ParsedData.AvgRate)
title('Average Rate at Every Second')
ylabel('Average Rate (Hz)')

subplot(5,2,[3,4])
plot(ParsedData.MinTime); hold on
plot(ParsedData.MaxTime); hold off
title('Arrival Time')
legend('Min Time','Max Time','Location','north','Orientation','horizontal')

subplot(5,2,[5,6])
plot(abs((ParsedData.MinTime-ExpectedPeriod)/ExpectedPeriod)); hold on
plot(abs((ParsedData.MaxTime-ExpectedPeriod)/ExpectedPeriod)); hold off
title('|Arrival Time - Expected| / Expected')
legend('Min Time','Max Time','Location','north','Orientation','horizontal')

subplot(5,2,[7,8])
histogram(abs((ParsedData.MinTime-ExpectedPeriod)/ExpectedPeriod),...
    'Normalization','probability','BinWidth',0.01); hold on
histogram(abs((ParsedData.MaxTime-ExpectedPeriod)/ExpectedPeriod),...
    'Normalization','probability','BinWidth',0.01); hold off
title('Histogram for |Arrival Time - Expected| / Expected')
ylabel('Probability')

subplot(5,2,9)
boxplot(ParsedData{:,1},'OutlierSize',2)
title('Overall Rate Stats')
ylabel('Average Rate (Hz)')
set(gca,'XTickLabel',{' '})

subplot(5,2,10)
boxplot(ParsedData{:,2:end-1},{'Min Time','Max Time','SD'},'OutlierSize',2)
title('Overall Time Stats')
ylabel('Time Interval (s)')

% plot(ParsedData.MinTime-ExpectedPeriod); hold on
% plot(ParsedData.MaxTime-ExpectedPeriod); hold off
% title('Min and Max Times minus Expected Period')
% xlabel('Elapsed Time (s)')
% ylabel('Jitter (s)')
% legend('Min Time','Max Time')

%% Save data to file
disp('Saving mat file...');
save(Filename,'ExpectedPeriod','Filename','Format','ParsedData');

    %% Save figure
savefig(Filename);

