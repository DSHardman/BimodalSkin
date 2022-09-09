function [trainmeans, valmeans, testmeans, errors, pred, target, net] = sensorTrain(inp, out, sens_size,...
                                inp_sens, out_pred, fclayers, normalise, ratio, figs)
% Train feedforward neural network to predict localisation, depth, and
% temperature of probing based on strain and temperature feedback

    % INPUTS
    % inp: filtered/extracted sensor data
    % out: [x y depth temp] targets
    % sens_size: 's', 'm', or 'l' corresponding to small, medium, large grids
    % inp_sens: 's', 't', 'b' - train network with strain/temp/both inputs
    % out_pred: predict [localisation depth temp] boolean values
    % fclayers: hidden layer sizes - all fully connected, with tanh activation
    % normalise: should given inputs be normalised before training?
    % ratio: [train val test] ratios: must add to 1
    % figs: plot figures/training progress?

    %% Normalise input values
    if normalise
        inp = (inp - mean(inp))./std(inp);
    end
    
    %% Convert input based on desired training type
    switch inp_sens
        case 's' % strain
            samples = size(inp,2)/16;
            temporary_inp = zeros(size(inp,1), samples*8);
            for i = 1:8
                temporary_inp(:,(i-1)*samples+1:i*samples) = inp(:,(2*i-1)*samples+1:(2*i)*samples);
            end
            inp = temporary_inp;
        case 't' % temperature
            samples = size(inp,2)/16;
            temporary_inp = zeros(size(inp,1), samples*8);
            for i = 1:8
                temporary_inp(:,(i-1)*samples+1:i*samples) = inp(:,(2*i-2)*samples+1:(2*i-1)*samples);
            end
            inp = temporary_inp;
        case 'b' % both
            % no changes
        otherwise
            fprintf('Invalid input sensors');
    end

    %% Standardize outputs between 0 and 1
    switch sens_size
        case 's'
            out(:,1) = out(:,1)./25;
            out(:,2) = out(:,2)./20;
        case 'm'
            out(:,1) = out(:,1)./45;
            out(:,2) = out(:,2)./40;
        case 'l'
            out(:,1) = out(:,1)./50;
            out(:,2) = out(:,2)./50;
        otherwise
            fprintf('Invalid Size');
    end
    out(:,3) = (out(:,3)-1)./3; % depths 1-4 mm
    out(:,4) = (out(:,4)-20)./80; % Temperatures mostly 20-100 C

    %% Convert output based on desired prediction type
    temporary_out = [];
    if out_pred(1)
        temporary_out = [temporary_out out(:, 1:2)];
    end
    if out_pred(2)
        temporary_out = [temporary_out out(:, 3)];
    end
    if out_pred(3)
        temporary_out = [temporary_out out(:, 4)];
    end
    positions = out(:,1:2);
    out = temporary_out;
    
    %% Extract training, validation, and test data
    assert(sum(ratio)==1);

    % Training data
    P=randperm(length(inp)); % randomise order first to avoid temporal effects
    XTrain=inp(P(1:round(ratio(1)*length(inp))),:);
    YTrain=out(P(1:round(ratio(1)*length(inp))),:);
    TrainPositions = positions(P(1:round(ratio(1)*length(inp))),:);
    
    % Validation data
    XVal=inp(P(round(ratio(1)*length(inp))+1:round(sum(ratio(1:2))*length(inp))),:);
    YVal=out(P(round(ratio(1)*length(inp))+1:round(sum(ratio(1:2))*length(inp))),:);
    ValPositions = positions(P(round(ratio(1)*length(inp))+1:round(sum(ratio(1:2))*length(inp))),:);
    
    % Test data
    XTest=inp(P(round(sum(ratio(1:2))*length(inp)+1):end),:);
    YTest=out(P(round(sum(ratio(1:2))*length(inp)+1):end),:);
    TestPositions = positions(P(round(sum(ratio(1:2))*length(inp)+1):end),:);


    %% Build network architecture and training options
    layers = [featureInputLayer(size(XTrain,2),"Name","featureinput")];
    for i = 1:length(fclayers)
        fcname = "fc"+string(i);
        tanhname = "tanh"+string(i);
        layers = [layers fullyConnectedLayer(fclayers(i),...
            "Name", fcname) tanhLayer("Name",tanhname)];
    end
    layers = [layers fullyConnectedLayer(size(out, 2),"Name","fc_out")...
        regressionLayer("Name","regressionoutput")];
    
    if figs
        plotstring = 'training-progress';
    else
        plotstring = 'none';
    end

    opts = trainingOptions('sgdm', ...
        'MaxEpochs',5000, ...
        'MiniBatchSize', 500,...
         'ValidationData',{XVal,YVal}, ...
        'ValidationFrequency',30, ...
        'GradientThreshold',1000, ...
        'ValidationPatience',100,...
        'InitialLearnRate',0.05*0.4, ...
        'LearnRateSchedule','piecewise', ...
        'LearnRateDropPeriod',500, ...
        'LearnRateDropFactor', 0.05, ...
        'Verbose',0, ...
        'Plots',plotstring, 'ExecutionEnvironment', 'gpu');
    
    %% Train network
    [net, ~] = trainNetwork(XTrain,YTrain,layers, opts);

    %% Calculate and return mean errors for training, validation, and test sets
    [errors, ~, ~] = calculateErrors(XTrain, YTrain, TrainPositions, net, sens_size, out_pred, figs);
    if figs
        sgtitle('Train');
    end
    trainmeans = mean(abs(errors));
    [errors, ~, ~] = calculateErrors(XVal, YVal, ValPositions, net, sens_size, out_pred, figs);
    if figs
        sgtitle('Validation');
    end
    valmeans = mean(abs(errors));
    [errors, pred, target] = calculateErrors(XTest, YTest, TestPositions, net, sens_size, out_pred, figs);
    if figs
        sgtitle('Test');
    end
    testmeans = mean(abs(errors));
end