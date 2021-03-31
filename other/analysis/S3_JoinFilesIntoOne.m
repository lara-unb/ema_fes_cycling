% 
% MATLAB code - EMA Matrix Experiments
% 2021-03-29
% Lucas de Macedo Pinheiro
% 
%   Join all MAT files into one.
%

% Open window for file selection
disp('Select the matfiles...');
Files = uigetfile('*.mat','Select The Matfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end
JointData = struct();

%%
for w = 1:length(Files)
    %% Import file
    CurrentFileName = Files{w};
    fprintf('\n\nImporting "%s" mat file...\n',CurrentFileName);
    File = load(CurrentFileName);
    CurrentFileName = CurrentFileName(1:end-4);
    
    %% Join the data
    FieldList = fieldnames(File);
    for i = 1:length(FieldList)
        fprintf('Adding "%s"\n',FieldList{i});
        % Join method depends on class of data structure
        if isa(File.(FieldList{i}),'timeseries')
            if ~isfield(JointData,FieldList{i})
                % Create a new field if not present yet
                JointData.(FieldList{i}) = timeseries();
            end
            JointData.(FieldList{i}) = timeseries([JointData.(FieldList{i}).Data; File.(FieldList{i}).Data],...
                [JointData.(FieldList{i}).Time; File.(FieldList{i}).Time]);
        elseif isa(File.(FieldList{i}),'table')
            if ~isfield(JointData,FieldList{i})
                % Create a new field if not present yet
                JointData.(FieldList{i}) = table();
            end
            JointData.(FieldList{i}) = [JointData.(FieldList{i});File.(FieldList{i})];
            % Sort table to have ascending time
            JointData.(FieldList{i}) = sortrows(JointData.(FieldList{i}), 'Time');
        % If not dealt with before gather in a substruct
        else
            if ~isfield(JointData,FieldList{i})
                % Create a new field if not present yet
                JointData.(FieldList{i}) = struct([FieldList{i} num2str(w)],File.(FieldList{i}));
            else
                JointData.(FieldList{i}).([FieldList{i} num2str(w)]) = File.(FieldList{i});
            end
        end
    end
end

%% Compute and store the time offset
fprintf('\nAdding TimeOffset\n');
try  % Check if StimMatrixRaw is present, if not it is not a matrix dataset
    % Time offset to start with zero
    JointData.TimeOffset = min([JointData.CadenceRaw.Time(1),...
        JointData.DistanceRaw.Time(1),JointData.PedalAngleRaw.Time(1),...
        JointData.SpeedRaw.Time(1),JointData.StimCurrentRaw.Time(1),...
        JointData.StimMatrixRaw.Time(1),JointData.StimPulseWidthRaw.Time(1)]);
catch e  % Carry on considering a conventional dataset
    if strcmp(e.identifier,'MATLAB:nonExistentField')
        JointData.TimeOffset = min([JointData.CadenceRaw.Time(1),...
            JointData.DistanceRaw.Time(1),JointData.PedalAngleRaw.Time(1),...
            JointData.SpeedRaw.Time(1),JointData.StimCurrentRaw.Time(1),...
            JointData.StimPulseWidthRaw.Time(1)]);
    else  % Pass on the error
        rethrow(e)
    end
end

%% Save data to file
disp('Saving mat file...');
save(['_' JointData.Filename.Filename1(1:end-1) 'all'],'-struct','JointData');
