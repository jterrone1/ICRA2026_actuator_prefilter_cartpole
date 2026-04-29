function error = evaluate_koopman_prediction(X_t, X_tp1, koop_model)

A = koop_model.A;
B = koop_model.B;
centers = koop_model.centers;
epsilon = koop_model.epsilon;

yt = lift(centers, epsilon, X_t)'; % lift data sets
ytp1 = lift(centers, epsilon, X_tp1)';

% error from training data set
error_vec = vecnorm((ytp1 - A*yt), 2, 1); 
error = mean(error_vec);

end