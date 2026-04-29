function [z_out, u_out] = simulate_cartpole_controller(koop_model, p, rng_val)

    addpath(genpath('C:\Users\jasmi\OneDrive\Desktop\rimless_matlab\cartpole_torsional_ICRA\'));



    %% Perform Dynamic simulation    
    dt = 0.01;
    tf = 12;
    num_steps = floor(tf/dt);
    tspan = linspace(0, tf, num_steps); 
    rng(rng_val)
    left_loc = p(7);
    right_loc = p(8);

    %{
    if left_loc ~= 0 && right_loc ~=0
        lambda = [0;0];
        while norm(lambda) == 0 
            z0 = [0.2*(rand(1)-0.5); (pi/2)*(rand(1)-0.5); 2*(rand(1)-0.5); 2*(rand(1)-0.5); (rand(1)-0.5)];
            lambda = contact_force(z0, p);
        end
    end
    %}

    z0 = [0.2*(rand(1)-0.5); (pi/2)*(rand(1)-0.5); 2*(rand(1)-0.5); 2*(rand(1)-0.5); rand(1)];
    
    z_out = zeros(5,num_steps);
    u_out = zeros(1,num_steps);
    z_out(:,1) = z0;
    for i=1:num_steps-1
        
        [dz, u, ~] = dynamics(z_out(:,i), p, 2, koop_model);

        z_out(:,i+1) = z_out(:,i) + dz*dt;
        %z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:end,i+1)*dt;
        u_out(:, i) = u;

    end
    

    %% Animate Solution

    %{
   

    myVideo = VideoWriter('Passive2'); %open video file
    myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
    open(myVideo)

    limit1 = [min(z_out(1,:))-3*l; 0];
    limit2 = [max(z_out(1,:))+3*l; 0];

    limit1 = [-3*l; 0];
    limit2 = [3*l; 0];

    figure(2); clf;
        % Prepare plot handles
    
    hold on
    cart = plot([0],[0],'LineWidth',5);
    pole = plot([0],[0],'LineWidth',2);
    xlabel('x')
    ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    %axis([min(limit1(1), limit2(1)) max(limit1(1), limit2(1)) min(limit1(2), limit2(2))-2*l max(limit1(2), limit2(2))+5*l]);
    skip_frame = 10;


    line([limit1(1),limit2(1)],[limit1(2),limit2(2)])


    line([-0.1 -0.1], [0, 0.7], 'Color','k', 'LineWidth',0.5);
    line([0.1 0.1], [0, 0.7], 'Color','k', 'LineWidth',0.5);
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

        pause(.01)
    end

    close(myVideo)

    clf
    %}

end