%% This is for strategy 1 - const. PWM for free-loaded motor
%% Setup
clear *;
close all;
HebiLookup.initialize();

familyName = 'synthesis_lectures';
moduleNames = 'motor_3';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Command Loop (Effort)
% The commmand struct has fields for position, velocity, and effort.  
% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 
                       
% Starts logging in the background
group.startLog( 'dir', 'logs' ); 

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency                 
    fbk = group.getNextFeedbackFull();  
                
    cmd.velocity = 1.0;
    group.send(cmd);
   
end


% Stop logging and plot the effort data using helper functions
log = group.stopLogFull();
%HebiUtils.plotLogs( log, 'effort' );


% Velocity against time
figure(1); scatter(log.time, log.motorVelocity); grid;
title('Velocity against time'); 
xlabel('Time(s)'); ylabel('Velocity(rads^-1)');
%text(x, y, t.Name, 'Vert','bottom', 'Horiz','left', 'FontSize',7)
%set(gca, 'XScale','log', 'YScale','log')

figure(2); scatter(log.time,log.windingCurrent); grid;
title('windingCurrent against time');
xlabel('Time(s)'); ylabel('windingCurrent(A)');


%Result from the const_vI_free test
damping_coe = 2.652e-05;
velo_const = 0.00926;


%differntiation of motorVelocity with respect to time
diffVelo_time = diff(log.motorVelocity)./diff(log.time);
diffVelo_time = [0; diffVelo_time];
%motor inertia
figure(3); scatter(diffVelo_time, ...
    velo_const*log.windingCurrent - damping_coe*log.motorVelocity);grid;
% linear fitting
linearM1 = fitlm(diffVelo_time, ...
    velo_const*log.windingCurrent - damping_coe*log.motorVelocity);
plot (linearM1);
title('motor inertia');
xlabel('angular accleration(rad/s^2)'); ylabel('output-damping Torque(Nm)');
inertia = linearM1.Coefficients.Estimate(2);
