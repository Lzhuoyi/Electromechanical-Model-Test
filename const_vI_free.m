%% This is for strategy 4 - const. velocity and current for free-loaded motor
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


%Armature constant
% 1 rads^-1 = 9.55 rpm
figure(3); scatter(log.motorVelocity, ...
    log.voltage.*log.pwmCmd-log.windingCurrent.*5.53);grid;
% linear fitting
linearM1 = fitlm(log.motorVelocity, ...
    log.voltage.*log.pwmCmd-log.windingCurrent.*5.53);
plot (linearM1);
title('Armature constant');
xlabel('motor velocity(rad/s)'); ylabel('motor emf (v)');
velo_const = linearM1.Coefficients.Estimate(2);
 

%Damping ceofficient
figure(4); scatter(log.windingCurrent,log.motorVelocity); grid;
% linear fitting
linearM2 = fitlm(log.windingCurrent,log.motorVelocity);
plot (linearM2);
title('Velocity against windingCurrent');
xlabel('windingCurrent(A)'); ylabel('motor velocity(rad/s)');
damping_coe = -velo_const/linearM2.Coefficients.Estimate(2);



