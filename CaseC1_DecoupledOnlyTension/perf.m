load('workspace_Decoupled.mat')

for i = 1:100
    env=CDPRENV(inputs);
%rng(0)
    inputs.train = true;
    
    simOpts = rlSimulationOptions('MaxSteps',maxsteps);
    experience = sim(env,agent,simOpts);

    X = experience.Observation.CurrentEndEffectorPosition_Velocity_Ls01Ls02Ls03Ls04.Data;
%X = reshape(X,[301,6]);
    X = squeeze(X);

    ErrorX = X(7,:);
    ErrorY = X(8,:);
    track_error = sqrt(ErrorX.^2 + ErrorY.^2);
    track_error_sq = track_error.^2;

    RMSE_D(i) = sqrt(sum(track_error_sq)/200);


    K_D(i)=sum(env.K_vec)/200;
    M_D(i)= sum(env.M_vec)/200;

end

%  RMSE_D_scaled = rescale(RMSE_D,0,1);
% K_D = K_D./2;
% M_D = M_D./2;

[S1_D,M1_D] = std(RMSE_D);
[S2_D,M2_D] = std(K_D);
[S3_D,M3_D] = std(M_D);