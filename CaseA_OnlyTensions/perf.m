%% Only Tension

clc
clear

load('workspace_OT.mat')

for i = 1:100
    env=CDPRENV(inputs);
%rng(0)
    inputs.train = true;
    
    simOpts = rlSimulationOptions('MaxSteps',maxsteps);
    experience = sim(env,agent,simOpts);

    X = experience.Observation.ActualEndEffectorPosition_Velocity_PositionError_VelocityError.Data;
%X = reshape(X,[301,6]);
    X = squeeze(X);

    ErrorX = X(7,:);
    ErrorY = X(8,:);
    track_error = sqrt(ErrorX.^2 + ErrorY.^2);
    track_error_sq = track_error.^2;

    RMSE_OT(i) = sqrt(sum(track_error_sq)/200);

    K_OT(i)=sum(env.K_vec)/200;
    M_OT(i)= sum(env.M_vec)/200;

end

 %RMSE_OT_scaled = rescale(RMSE_OT,0,1);

%K_OT = K_OT;
%M_OT = M_OT./2;

[S1_OT,M1_OT] = std(RMSE_OT);
[S2_OT,M2_OT] = std(K_OT);
[S3_OT,M3_OT] = std(M_OT);
