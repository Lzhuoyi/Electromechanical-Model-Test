% Send effort commands, log in the background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

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

% Parameters for sin/cos function
freqHz = 0.5;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = 0.1;                % [Nm]

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency                 
    fbk = group.getNextFeedbackFull();  
                
    %cmd.velocity = amp;
    cmd.effort = amp;% * sin( freq * toc(timer) )+0.1;
    group.send(cmd);
   
end

% Stop logging and plot the effort data using helper functions
log = group.stopLogFull();
%HebiUtils.plotLogs( log, 'effort' );

diffVelo_time = diff(log.motorVelocity)./diff(log.time);
diffVelo_time = [0; diffVelo_time];
diffCurrent_time = diff(log.windingCurrent)./diff(log.time);
diffCurrent_time = [0; diffCurrent_time];


% Velocity against time
figure(1); scatter(log.time, log.motorVelocity); grid;
title('Velocity against time'); 
xlabel('Time(s)'); ylabel('Velocity(rads^-1)');
%text(x, y, t.Name, 'Vert','bottom', 'Horiz','left', 'FontSize',7)
%set(gca, 'XScale','log', 'YScale','log')

figure(2); scatter(log.time,log.windingCurrent); grid;
title('windingCurrent against time');
xlabel('Time(s)'); ylabel('windingCurrent(A)');

%Damping ceofficient
figure(3); scatter(log.windingCurrent,log.motorVelocity); grid;
% linear fitting
linearM1 = fitlm(log.windingCurrent,log.motorVelocity);
plot (linearM1);
title('Velocity against windingCurrent');
xlabel('windingCurrent(A)'); ylabel('Velocity(rpm)');

%Armature constant
% 1 rads^-1 = 9.55 rpm
figure(4); scatter(log.motorVelocity.*9.55, ...
    log.voltage.*log.pwmCmd-log.windingCurrent.*5.53);grid;
% linear fitting
linearM2 = fitlm(log.motorVelocity.*9.55, ...
    log.voltage.*log.pwmCmd-log.windingCurrent.*5.53);
plot (linearM2);
title('damping coefficient');
xlabel('velocity(rpm)'); ylabel('Voltage(V)');

%motor inertia
kv = 3.8*2*pi/60;
figure(5); scatter(diffVelo_time, ...
    kv*log.windingCurrent - 0.00268*log.motorVelocity);grid;
% linear fitting
linearM3 = fitlm(diffVelo_time, ...
    kv*log.windingCurrent - 0.00268*log.motorVelocity);
plot (linearM3);
title('motor inertia');
%xlabel('velocity(rpm)'); ylabel('Voltage(V)');

%inductance
% figure(5); scatter(diffCurrent_time, ...
%     log.voltage.*log.pwmCmd-log.windingCurrent.*5.53);grid;
% % linear fitting
% linearM4 = fitlm(diffCurrent_time, ...
%     log.voltage.*log.pwmCmd-log.windingCurrent.*5.53);
% plot (linearM4);
% title('inductance');