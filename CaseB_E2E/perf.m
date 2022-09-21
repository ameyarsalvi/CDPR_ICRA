clc
clear

load('workspace_E2E.mat')
rng(0)
for i = 1:100
    env=CDPRENV(inputs);
%rng(0)
    inputs.train = true;
    
    simOpts = rlSimulationOptions('MaxSteps',maxsteps);
    experience = sim(env,agent,simOpts);

    X = experience.Observation.ActualEndEffectorPosition_Velocity_PositionError_VelocityError_.Data;
%X = reshape(X,[301,6]);
    X = squeeze(X);

    ErrorX = X(5,:);
    ErrorY = X(6,:);
    track_error = sqrt(ErrorX.^2 + ErrorY.^2);
    track_error_sq = track_error.^2;

    RMSE_E(i) = sqrt(sum(track_error_sq)/200);

    K_E(i)=sum(X(11,:))/200;
    M_E(i)= sum(X(12,:))/200;

end

 %RMSE_E_scaled = rescale(RMSE_E,0,1);

%  K_E = K_E.;
%  M_E = M_E./2;
[S1_E,M1_E] = std(RMSE_E);
[S2_E,M2_E] = std(K_E);
[S3_E,M3_E] = std(M_E);