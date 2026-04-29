function lambda = contact_force(z, p)
        keypoints = keypoints_cartpole(z,p);
        rP = keypoints(:,2);

        k = p(5);

        if rP(1) < p(7)
            lambda2 = max(0, k*(p(7)-rP(1)));
            lambda1 = 0;
        elseif rP(1) > p(8)
            lambda1 = max(0, -k*(-rP(1) + p(8)));
            lambda2 = 0;
        else
            lambda2 = 0;
            lambda1 = 0;
        end
        

        if p(7) == 0 && p(8) == 0
            lambda1 = 0;
            lambda2 = 0;
        end

        lambda = [lambda1, lambda2]';

        % Compare values to LC solver, should be identical

        %{
        Fc = 0.1*eye(2);  %STIFFNESS PARAMETER HERE
        alpha = pi/2;
        d1 = 0.1;
        d2 = -0.1;

        p = [z(1) - 0.5*sin(z(2)); 0.5*cos(z(2))];

        phi1 = 0.1*sin(alpha) - p(1)*sin(alpha) + p(2)*cos(alpha);
        phi2 = p(1)*sin(alpha) - (-0.1)*sin(alpha) + p(2)*cos(alpha);
        
        %calculate the contact force lambda
        lambda = pathlcp(Fc,[phi1;phi2]);
        %}

end