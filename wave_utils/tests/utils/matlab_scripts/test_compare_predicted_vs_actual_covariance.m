disp 'TEST: Compare predicted vs actual covariance'
disp 'number of samples: 10000'
disp 'pose1 and pose2 have 0 mean pose and the same covariance matrices'
disp 'Testing...'

num_samples = 10000;

p1 = [0 0 0 0 0 0];
p1_cov = [   1,  0.1, -0.2,  0.1,  0.3,  0.1;
           0.1,    1, -0.3, -0.4,  0.1,  0.3;
          -0.2, -0.3,    1,  0.3, -0.4,  0.4;
           0.1, -0.4,  0.3,    1, -0.3,  0.5;
           0.3,  0.1, -0.4, -0.3,    1,  0.3;
           0.1,  0.3,  0.4,  0.5,  0.3,    1];
p1_cov = 1e-3 * (p1_cov' * p1_cov);

p2 = [0 0 0 0 0 0];
p2_cov = p1_cov;

[predicted_cov, actual_cov, error_plot] = compare_predicted_vs_actual_covariance(p1, p1_cov, p2, p2_cov, num_samples);
predicted_cov = predicted_cov
actual_cov = actual_cov
error = norm(predicted_cov - actual_cov)/norm(actual_cov)

if (abs(predicted_cov - actual_cov) < 1e-4*ones(6,6))
    disp 'Predicted and actual covariance match';
else
    disp 'Predicted and actual covariance do NOT match';
end

plot(error_plot(:,1), error_plot(:,2))
xlabel('Number of iterations')
ylabel('Norm-2 Error of predicted matrix')
title('Norm-2 Error of predicted wrt actual covariance matrix')