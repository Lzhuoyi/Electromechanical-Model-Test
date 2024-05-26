%% This is for strategy 1 - const. PWM for blocked motor
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


% Armarture resistance
figure(3); scatter(log.windingCurrent,log.voltage.*log.pwmCmd); grid;
% linear fitting
linearM1 = fitlm(log.windingCurrent,log.voltage.*log.pwmCmd);
plot (linearM1);
title('Voltage against windingCurrent');
xlabel('windingCurrent(A)'); ylabel('Voltage(V)');
resistance = linearM1.Coefficients.Estimate(2);


%differentiation of winding current with respect to time
diffCurrent_time = diff(log.windingCurrent)./diff(log.time);
diffCurrent_time = [0; diffCurrent_time];
%Wingding inductance
figure(4); scatter(diffCurrent_time, ...
    log.voltage.*log.pwmCmd-log.windingCurrent.*resistance);grid;
% linear fitting
linearM2 = fitlm(diffCurrent_time, ...
    log.voltage.*log.pwmCmd-log.windingCurrent.*resistance);
plot (linearM2);
title('inductance');
xlabel('angular accleration(rad/s^2)'); ylabel('output-damping Torque(Nm)');
inductance = linearM2.Coefficients.Estimate(2);