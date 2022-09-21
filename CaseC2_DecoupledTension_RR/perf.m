%load('workspace_Decoupled.mat')

for i = 1:2
    env=CDPRENV(inputs);
%rng(0)
    inputs.train = true;
    
    simOpts = rlSimulationOptions('MaxSteps',maxsteps);
    experience = sim(env,agent,simOpts);

    X = experience.Observation.CurrentEndEffectorPosition_Velocity_Ls01Ls02Ls03Ls04.Data;
%X = reshape(X,[301,6]);
    X = squeeze(X);

    ErrorX = X(5,:);
    ErrorY = X(6,:);
    track_error = sqrt(ErrorX.^2 + ErrorY.^2);
    track_error_sq = track_error.^2;

    RMSE_OTT(i) = sqrt(sum(track_error_sq)/200);


    K_OTT(i)=sum(env.K_vec)/200;
    M_OTT(i)= sum(env.M_vec)/200;


end

%  RMSE_D_scaled = rescale(RMSE_D,0,1);
% K_D = K_D./2;
% M_D = M_D./2;

[S1_OTT,M1_OTT] = std(RMSE_OTT);
[S2_OTT,M2_OTT] = std(K_OTT);
[S3_OTT,M3_OTT] = std(M_OTT);