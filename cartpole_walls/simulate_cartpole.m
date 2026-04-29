function simulate_cartpole()

    clf
    clear
    clc

    %% Define fixed paramters
    
    mP = 0.1;
    mC = 1;
    l = 0.5;
    g = 9.81;
    k = 10;
    %k_tors = 2;
    %b_tors = 0.05;

    tau = 2; % make sure this always reflects the Koopman model
    rng_val =  85;

    left_loc = -0.2; % -100 turns off wall
    right_loc = 0.2; % 100 turns of wall
    d = abs(left_loc);

    p   = [mP mC l g k tau left_loc right_loc]';        % parameters
    

    % load in koopman model/lqr controller 

    baseDir = fileparts(mfilename('fullpath'));
    modelPath = fullfile(baseDir, 'koopman_models', '0theta_best_tau_2_obs_25_eps_2.1_-0.2_wall_0.2.mat');
    %modelPath = fullfile(baseDir, 'koopman_models', 'tau_2_obs_3_eps_6.mat');
    
    load(modelPath, 'koop_model');
    


    %% Perform Dynamic simulation    
    dt = 0.01;
    tf = 6;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    rng(rng_val)
    
    z0 = [0.2*(rand(1)-0.5); 0*(pi/2)*(rand(1)-0.5); 2*(rand(1)-0.5); 2*(rand(1)-0.5); (rand(1)-0.5)];

    
    if left_loc ~= 0 && right_loc ~=0
        lambda = [0;0];
        while norm(lambda) ~= 0 
            z0 = [0.2*(rand(1)-0.5); (pi/2)*(rand(1)-0.5); 2*(rand(1)-0.5); 2*(rand(1)-0.5); (rand(1)-0.5)];
            lambda = contact_force(z0, p);
        end
    end
    

    %z0 = [0.1; -0.01; 0.5; 0; 0.1];

    % z0 = [x, th, dx, dth, F]

    
    z_out = zeros(5,num_steps);
    z_out(:,1) = z0;
    for i=1:num_steps-1
        
        [dz, u, ~] = dynamics(z_out(:,i), p, 2, koop_model);
        %[dz, u, ~] = dynamics(z_out(:,i), p, 0);
        z_out(:,i+1) = z_out(:,i) + dz*dt;

        %z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:end,i+1)*dt;

    end
    final_state = z_out(:,end);
    %writematrix(z_out,"cck_lqr_nowallsnospring_rng85.csv") 



    %% Animate Solution

    %{
    myVideo = VideoWriter('test'); %open video file
    myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
    open(myVideo)

    limit1 = [min(z_out(1,:))-l; 0];
    limit2 = [max(z_out(1,:))+l; 0];

    limit1 = [-l; 0];
    limit2 = [l; 0];

    figure(2); clf;
        % Prepare plot handles
    
    hold on
    cart = plot([0],[0],'LineWidth',5);
    pole = plot([0],[0],'LineWidth', 3);
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([min(limit1(1), limit2(1)) max(limit1(1), limit2(1)) min(limit1(2), limit2(2))-0.5*l max(limit1(2), limit2(2))+2*l]);
    skip_frame = 10;


    line([limit1(1),limit2(1)],[limit1(2),limit2(2)])

    if left_loc ~=0 && left_loc > -100
        line([left_loc left_loc], [0, 0.7], 'Color','k', 'LineWidth',0.5);

    end

    if right_loc ~= 0 && right_loc < 100
        line([right_loc right_loc], [0, 0.7], 'Color','k', 'LineWidth',0.5);
    end

    %line([limit1(1),limit2(1)],[limit1(2),limit2(2)])


    %line([-5,5],[-1,-1])

    %Step through and update animation
    for i=1:num_steps
        
        if mod(i, skip_frame)
           continue
        end

        % interpolate to get state at current time.
        t = tspan(i);

        z = z_out(:,i);
        keypoints = keypoints_cartpole(z,p);

        rC = keypoints(:,1);
        rP = keypoints(:,2);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        

        % Plot Pole
        set(cart,'XData' , [rC(1)-0.05 rC(1)+0.05] );
        set(cart,'YData' , [rC(2) rC(2)] );

        set(pole,'XData' , [rC(1) rP(1)] );
        set(pole,'YData' , [rC(2) rP(2)] );


        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);

        pause(.03)
    end

    close(myVideo)

    clf
    %}

    %% Fancier Animation

    
    
    colors = parula(4);
    myVideo = VideoWriter('test'); 
    myVideo = VideoWriter(sprintf('Cart-Pole_KLQR_w_walls_lastvid_r%.0f', [rng_val]), 'MPEG-4'); 
    myVideo.FrameRate = 60;
    open(myVideo)
    
    limit1 = [-1.2*l; 0];
    limit2 = [1.2*l; 0];
    
    figure(2); clf;
    hold on
    
    % Ground
    line([limit1(1),limit2(1)], [limit1(2),limit2(2)], 'Color','k');
    
    
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([min(limit1(1), limit2(1)) max(limit1(1), limit2(1)) ...
          min(limit1(2), limit2(2))-0.5*l max(limit1(2), limit2(2))+1.5*l]);
    
    skip_frame = 10;
    
    cart_w = 0.3; 
    cart_h = 0.15;
    wheel_d = cart_h/2; 
    
    surf_left_rest  = -d;
    surf_right_rest =  d;
    
    anchor_left  = -3*d;
    anchor_right =  3*d;
    
    y_offset = 0.2; % vertical shift for springs & surfaces
    y_mid = l/2 + y_offset;

    cart = rectangle('Position',[0 0 cart_w cart_h], ...
                     'FaceColor', 'w','EdgeColor','k','LineWidth',4);
    pole = plot([0 0],[0 0],'k','LineWidth',4);
    point_mass  = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',12);
    
    wheel1 = rectangle('Position',[0 0 wheel_d wheel_d], ...
                       'Curvature',[1 1],'FaceColor','w','EdgeColor','k','LineWidth',4);
    wheel2 = rectangle('Position',[0 0 wheel_d wheel_d], ...
                       'Curvature',[1 1],'FaceColor','w','EdgeColor','k','LineWidth',4);
    
    % Springs
    spring_left  = plot(nan,nan,'k','LineWidth',2);
    spring_right = plot(nan,nan,'k','LineWidth',2);
    
    % Surfaces
    surface_left  = plot([surf_left_rest surf_left_rest],[y_offset l+y_offset],'k','LineWidth',3);
    surface_right = plot([surf_right_rest surf_right_rest],[y_offset l+y_offset],'k','LineWidth',3);
    
    % Spring generator
    make_spring = @(x1,y1,x2,y2,n) deal( ...
        linspace(x1,x2,n), ...
        y1 + 0.05*sawtooth(linspace(0,6*pi,n),0.5) );
    
    % ---- Animation loop ----
    for i=1:num_steps
        %if mod(i, skip_frame)
        %    continue
        %end
    
        t = tspan(i);
        z = z_out(:,i);
        keypoints = keypoints_cartpole(z,p);
    
        rC = keypoints(:,1); % cart location
        rP = keypoints(:,2); % pole tip
    
        % set surfaces at rest position
        surf_left = surf_left_rest;
        surf_right = surf_right_rest;
    
        % flags
        collide_left = false;
        collide_right = false;
    
        if rP(1) < surf_left_rest
            surf_left = min(rP(1), surf_left_rest); 
            collide_left = true;
        end

        if rP(1) > surf_right_rest
            surf_right = max(rP(1), surf_right_rest);
            collide_right = true;
        end
    
        [sx,sy] = make_spring(anchor_left,y_mid,surf_left,y_mid,40);
        set(spring_left,'XData',sx,'YData',sy, ...
                        'Color', collide_left * [1 0 0] + ~collide_left * [0 0 0]);
    
        [sx,sy] = make_spring(surf_right,y_mid,anchor_right,y_mid,40);
        set(spring_right,'XData',sx,'YData',sy, ...
                         'Color', collide_right * [1 0 0] + ~collide_right * [0 0 0]);
    
        set(surface_left,'XData',[surf_left surf_left],'YData',[y_offset l+y_offset]);
        set(surface_right,'XData',[surf_right surf_right],'YData',[y_offset l+y_offset]);
        
        set(cart,'Position',[rC(1)-cart_w/2, rC(2)-cart_h/2, cart_w, cart_h]);
        set(pole,'XData',[rC(1) rP(1)], 'YData',[rC(2) rP(2)]);
        set(point_mass,'XData',rP(1),'YData',rP(2));
    
        set(wheel1,'Position',[rC(1)-cart_w/4 - wheel_d/2-0.02, rC(2)-cart_h/2-wheel_d/2-0.025, wheel_d, wheel_d]);
        set(wheel2,'Position',[rC(1)+cart_w/4 - wheel_d/2+0.02, rC(2)-cart_h/2-wheel_d/2-0.025, wheel_d, wheel_d]);
    
        uistack(cart,'top');  % also ensure the point_mass stays on top
        uistack(pole,'top');
        uistack(point_mass,'top');  % also ensure the point_mass stays on top

        set(h_title,'String',  sprintf('CCK + LQR (with walls) and no torsional spring-damper, t=%.2f',[t]) ); % update title

        frame = getframe(gcf);
        writeVideo(myVideo, frame);
        pause(0.0001)
    end

    

end