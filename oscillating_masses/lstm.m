% Configure an LSTM to add to a model controller
%% Load Data
% Generated from the data_generator.m script, appended 
% matrices stored as text files, (101*N x chan)
move = readmatrix('move.txt');
reference = readmatrix('reference.txt');
response = readmatrix('response.txt');

% Align the channels correctly
response = response';
move = move';

% Set up indices to mark where sequences start and end
j = (1:100:101*200);

% Pre-allocation so MATLAB debugger takes a chill pill
data = cell(200,1);
labels = cell(200,1);

for i = 1:length(reference)
    % insert all sequences into a cell array for indexing
    data(i) = mat2cell(response(1:3,j(i):(j(i)+100)),3,101);
end

for k = 1:length(reference)
    % insert all optimal inputs to a cell aray for indexing
    labels(k) = mat2cell(move(1:2,j(k):(j(k)+100)),2,101);
end

numChannels = size(data{1},1);
numObservations = numel(data);
numResponses = size(labels{1},1);

%% Prepare Data for LSTM
[idxTrain,idxValidation,idxTest] = trainingPartitions(numObservations, ...
    [0.8 0.1 0.1]);

XTrain = data(idxTrain);
XValidation = data(idxValidation);
XTest = data(idxTest);

TTrain = labels(idxTrain);
TValidation = labels(idxValidation);
TTest = labels(idxTest);

%% Define LSTM Architecture
numHiddenUnits = 100;

layers = [ ...
    sequenceInputLayer(numChannels, Normalization="zscore")
    lstmLayer(numHiddenUnits, OutputMode="last")
    fullyConnectedLayer(numResponses)
    regressionLayer]

%% Configure Training Options
options = trainingOptions("adam", ...
    MaxEpochs=10, ...
    ValidationData={XValidation TValidation}, ...
    OutputNetwork="best-validation-loss", ...
    InitialLearnRate=0.005, ...
    SequenceLength="shortest", ...
    Plots="training-progress", ...
    Verbose= false);

%% Train
net = trainNetwork(XTrain, TTrain, layers, options);

%% Test
YTest = predict(net,XTest, SequenceLength="shortest");

%% Results Analysis