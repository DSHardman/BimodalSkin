function [errors, pred, target] = calculateErrors(X, target, positions, net, sens_size, out_pred, figs)
% Use given network and input/target to calculate and plot mean errors
% Called by sensorTrain.m - mirrors inputs
    
    %% Make predictions
    ypred = predict(net, X);

    %% Convert predictions to desired format
    pred = zeros(size(ypred));
    if out_pred(1)
        switch sens_size
            case 's'
                pred(:,1) = ypred(:,1).*25;
                pred(:,2) = ypred(:,2).*20;
                target(:,1) = target(:,1).*25;
                target(:,2) = target(:,2).*20;
            case 'm'
                pred(:,1) = ypred(:,1).*45;
                pred(:,2) = ypred(:,2).*40;
                target(:,1) = target(:,1).*45;
                target(:,2) = target(:,2).*40;
            case 'l'
                pred(:,1) = ypred(:,1).*50;
                pred(:,2) = ypred(:,2).*50;
                target(:,1) = target(:,1).*50;
                target(:,2) = target(:,2).*50;
            otherwise
                fprintf('Invalid Size');
        end
        if out_pred(2)
            pred(:,3) = ypred(:,3).*3 + 1;
            target(:,3) = target(:,3).*3 + 1;
        end
    end

    if out_pred(3)
        pred(:,end) = ypred(:,end).*80 + 20;
        target(:,end) = target(:,end).*80 + 20;
        if out_pred(2) && ~out_pred(1)
            pred(:,end-1) = ypred(:,end-1).*3 + 1;
            target(:,end-1) = target(:,end-1).*3 + 1;
        end
    end

    if out_pred(2) && ~out_pred(1) && ~out_pred(3)
        pred(:,1) = ypred(:,1).*3 + 1;
        target(:,1) = target(:,1).*3 + 1;
    end

    %% Calculate corresponding errors
    errors = pred - target;

    %% Scatter plots of errors, for localisation, depth, and temperature sensing
    if figs
        figure();
        if out_pred(1)
            subplot(1,3,1);
            localization = rssq(errors(:,1:2).');
            scatter(positions(:,1), positions(:,2), 40, localization, 'filled');
            colorbar();
            title('Localisation (mm)');
        end
        if out_pred(2)
            subplot(1,3,2);
            if out_pred(3)
                scatter(positions(:,1), positions(:,2), 40, abs(errors(:,end-1)), 'filled');
            else
                scatter(positions(:,1), positions(:,2), 40, abs(errors(:,end)), 'filled');
            end
            colorbar();
            title('Depth (mm)');
        end
        if out_pred(3)
            subplot(1,3,3);
            scatter(positions(:,1), positions(:,2), 40, abs(errors(:,end)), 'filled');
            colorbar();
            title('Temperature (^oC)');
        end
    end

    %% Return absolute localisation error rather than seperate x/y
    if out_pred(1)
        if size(errors,2) == 2
            errors = rssq(errors(:,1:2).');
        else
            errors = [rssq(errors(:,1:2).').' errors(:,3:end)];
        end
    end
end