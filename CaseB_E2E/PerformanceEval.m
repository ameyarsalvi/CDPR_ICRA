for i = 1:100
    env=CDPRENV(inputs);
%rng(0)
    inputs.train = true;
    
    simOpts = rlSimulationOptions('MaxSteps',maxsteps);
    experience = sim(env,agent,simOpts);

    X = experience.Observation.ActualEndEffectorPosition_Velocity_PositionError_VelocityError_.Data;
%X = reshape(X,[301,6]);
    X = squeeze(X);
    Act = experience.Action.DesiredCableTensions_DesiredSliderPositions.Data;
    Ac = squeeze(Act);

    ErrorX = X(7,:);
    ErrorY = X(8,:);

    RMSE(i) = sqrt(sum(ErrorX.^2 + ErrorY.^2))/1000;

    K(i)=sum(X(11,:))/1000;
    M(i)= sum(X(12,:))/1000;

end

 RMSE = rescale(RMSE,0,1);

[S1,M1] = std(RMSE);
[S2,M2] = std(K);
[S3,M3] = std(M);
BAR = [M1,M2,M3];
SD = [S1,S2,S3];


figure
bar(BAR)
