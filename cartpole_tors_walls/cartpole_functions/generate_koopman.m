function koop_model = generate_koopman(X_t, X_tp1, n, epsilon, p, nox_bool)

if nox_bool == 1
    [~, centers] = kmeans(X_tp1(:, 2:end), n, 'MaxIter',50000);
    
    yt = lift(centers, epsilon, X_t(:, 2:end))'; % lift data sets
    ytp1 = lift(centers, epsilon, X_tp1(:, 2:end))';
                
    A_temp = ytp1 * pinv(yt);
    
    A = zeros(n+5, n+5);
    A(1,1) = 1;
    A(1,3) = 0.01;% = dt, currently, this parameters is hardcoded in!!!
    A(2:end, 2:end) = A_temp;
else
    [~, centers] = kmeans(X_tp1, n);
    
    yt = lift(centers, epsilon, X_t)'; % lift data sets
    ytp1 = lift(centers, epsilon, X_tp1)';
        
        % yt = [w(1),  w(2),  w(3) ... w(N);
        %       q(1),  q(2),  q(3) ... q(N);
        %       g1(1), g1(2), g1(3)...g1(N);
        %         :     :      :        :
        %       gn(1), gn(2), gn(3)...gn(N)] 
        
    A = ytp1 * pinv(yt); % least squares solution, (n + 5) x (n + 5)
end

B = zeros(length(A), 1);
B(5) = 0.01/p(6);

koop_model.A = A;
koop_model.B = B;
koop_model.centers = centers;
koop_model.epsilon = epsilon;
koop_model.nox_bool = nox_bool;

end