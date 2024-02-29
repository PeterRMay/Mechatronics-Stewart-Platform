clear all; close all; clc;

%% ---------------INPUTS AND DISTURBANCES----------------
% sim parameters
TF = 20;
stepSize = 0.1;
n = TF/stepSize;
record = false;

t = linspace(0,TF,n);
nullSignal = zeros(3,n);
nullTimeseries = timeseries(nullSignal,t);
onesSignal = ones(3,n);
sineSignal = onesSignal;
cosineSignal = onesSignal;
for i = 1:length(sineSignal)
    sineSignal(:,i) = sineSignal(:,i) * sind(i*5);
    cosineSignal(:,i) = cosineSignal(:,i) * cosd(i*5);
end

% system inputs (should usually be 0)
% both 3x1 vectors
T_base = nullTimeseries;
Phi_base = nullTimeseries;

% system disturbances (these should be signals)
% these are measurements from IMU/Encoders
T_inertial_signal = sineSignal .* [2;0;0];
T_inertial = timeseries(T_inertial_signal,t);
Phi_inertial_signal = sineSignal .* [0;10;0] + cosineSignal.*[10;0;0];
Phi_inertial = timeseries(Phi_inertial_signal,t);

%% ------------------SYSTEM CONSTANTS----------------------
armLegRatio = 3/10;
platformBaseRatio = 0.6;
restingLegLength = 2;
platformRadius = 1;
servoMotorRange = 180;
servoTimeConstant = .354;
servoPeriod = 20e-3; % in ms
servoMaxPulse = 2.0; % in ms
servoMinPulse = 1.0; % in ms

DEN = [servoTimeConstant 1];

s = sqrt(restingLegLength^2 / (1+armLegRatio)); % length of rod
a = s * armLegRatio; % length of servo arm

% B[] distances to each leg, array of 3x1 location vectors
B1 = [platformRadius;0;0]; % ADD a name for length
legAngles = [-5,5,115,125,235,245];
B = zeros(3,6);
for i = 1:6
    B(:,i) = rotz(legAngles(:,i))*B1;
end

% P[] distances from origin of platform to the joints
P = B*platformBaseRatio; % make top platform smaller than bottom
defaultPlatformRotation = 60;
for i = 1:6
    P(:,i) = rotz(defaultPlatformRotation) * P(:,i); % rotate top platform
end
P = circshift(P,[0 1]); % shift matrix to shift which legs connect to which joints



%% --------------------DEFINE HOME POSITIONS----------------
beta = zeros(1,6); % angle between servo arm and x axis in the xy plane
% we add or subtract 90 to place servos arms perpendicular to vector
% through the center of the platform. This can be adjusted based on
% construction
for i = 1:2:6
    beta(i) = legAngles(i) - 90; % note this differs from the document, in the document beta is only for even legs
end
for i = 2:2:6
    beta(i) = legAngles(i) + 90;
end

% we now define the home height, h0, assuming 90 degrees between the arms
% and legs and no rotation or translational inputs
l0_squared = s^2+a^2; %default leg length using pythagoras theorem bc right angles
h0 = sqrt(l0_squared - (P(1,1) - B(1,1))^2 - (P(2,1) - B(2,1))^2) - P(3,1);
if isreal(h0) ~= true
%         warning("Legs too short given platform dimensions")
    errorFlag = true;
end

% next we define the home position of the servo motors
% this is a function purely of the desired default platform location and
% the leg lengths
Tdefault = [0;0;h0];
P_default = Tdefault + P;
alpha0 = ones(1,6) * servoCalc(P_default(:,1),B(:,1),sqrt(l0_squared),s,a,beta(1));
alpha0 = alpha0';

% set center of range of motion equal to alpha0
servoCenterOfRange = alpha0(1); % center of range of motion

% calculate maximum ranges of motion
l_max = s + a;
% l_min = 
alpha_max = servoCenterOfRange + servoMotorRange/2;
alpha_min = servoCenterOfRange - servoMotorRange/2;


%% ------------------SIMULINK--------------------

SimOut = sim('StewartPlatformSimulation.slx');

%add leg lengths as an output from inverse kinematics subsystem

P_base = SimOut.P_base.Data;
alphaActual = SimOut.actualServoAngles.Data;
alphaCommanded = squeeze(SimOut.commandedServoAngles.Data)';
simout_Phi_inertial = SimOut.Phi_inertial.Data;
commandedPosition = SimOut.commandedPosition.Data;




% %% -------------------PLOTTING----------------

video_filename = 'stewartplatformNew.avi';
n_pad_frames = 20; % number of starting/ending pad frames

seconds = 10;
framesPerSecond = length(alphaCommanded) / TF;
endVal = ceil(framesPerSecond*seconds);
interval = ceil(endVal / (30*seconds));

if (record)
    vid_obj = VideoWriter(video_filename); % create video object
    vid_obj.FrameRate = 30; % set frame rate [fps]
    open(vid_obj); % open video file
    for k=1:n_pad_frames
      writeVideo(vid_obj,getframe(gcf)); % write video frame
    end
    
    
    for q = 1:interval:endVal
        close all;
        %make rotation matrix
        phi_inertial = simout_Phi_inertial(1,1,q);
        theta_inertial = simout_Phi_inertial(2,1,q);
        psi_inertial = simout_Phi_inertial(3,1,q);
        R_Phi_inertial = rotz(psi_inertial)*roty(theta_inertial)*rotx(phi_inertial);
        % make plotting arrays
        P_base_plot = P_base(:,:,q);
        b_plot = B;
        for i = 1:6
            P_base_plot(:,i) = R_Phi_inertial*P_base_plot(:,i);
            b_plot(:,i) = R_Phi_inertial*b_plot(:,i);
        end
        
        P_base_plot(:,end+1) = P_base_plot(:,1);
        b_plot(:,end+1) = b_plot(:,1);
        
        plot3(P_base_plot(1,:),P_base_plot(2,:),P_base_plot(3,:),'-') % plot platform
        hold on
        plot3(b_plot(1,:),b_plot(2,:),b_plot(3,:),'-') % plot base
        axis equal
        
        % alpha = ones(1,6) * 0; % for entering manual alpha values
        servoArms = zeros(3,6);
        for i = 1:6
        %     plot3([b(1,i) P_base(1,i)], [b(2,i) P_base(2,i)], [b(3,i) P_base(3,i)]); % plot straight leg lengths
            servoArms(:,i) = B(:,i) + a * rotz(beta(i)) * roty(-alphaCommanded(q,i)) * [1; 0; 0]; % calculate arm positions
            servoArms(:,i) = R_Phi_inertial*servoArms(:,i);
            plotVector(b_plot(:,i), servoArms(:,i), 'black') % plot arm positions
            plotVector(servoArms(:,i), P_base_plot(:,i),'green') % plot leg positions
        end
        grid
        view(30,90)
        writeVideo(vid_obj,getframe(gcf));
        
    end
    
    
    for k=1:n_pad_frames
      writeVideo(vid_obj,getframe(gcf)); % write video frame
    end
    close(vid_obj); % close video file
else
    q = length(alphaActual);
    %make rotation matrix
    phi_inertial = simout_Phi_inertial(1,1,q);
    theta_inertial = simout_Phi_inertial(2,1,q);
    psi_inertial = simout_Phi_inertial(3,1,q);
    R_Phi_inertial = rotz(psi_inertial)*roty(theta_inertial)*rotx(phi_inertial);

    % make plotting arrays
    
        P_base_plot = P_base(:,:,q);
        b_plot = B;
        for i = 1:6
            P_base_plot(:,i) = R_Phi_inertial*P_base_plot(:,i);
            b_plot(:,i) = R_Phi_inertial*b_plot(:,i);
        end
        
        P_base_plot(:,end+1) = P_base_plot(:,1);
        b_plot(:,end+1) = b_plot(:,1);

        plot3(P_base_plot(1,:),P_base_plot(2,:),P_base_plot(3,:),'-') % plot platform
        hold on
        plot3(b_plot(1,:),b_plot(2,:),b_plot(3,:),'-') % plot base
        axis equal
        
        % alpha = ones(1,6) * 0; % for entering manual alpha values
        servoArms = zeros(3,6);
        for i = 1:6
        %     plot3([b(1,i) P_base(1,i)], [b(2,i) P_base(2,i)], [b(3,i) P_base(3,i)]); % plot straight leg lengths
            servoArms(:,i) = B(:,i) + a * rotz(beta(i)) * roty(-alphaCommanded(q,i)) * [1; 0; 0]; % calculate arm positions
            servoArms(:,i) = R_Phi_inertial*servoArms(:,i);
            plotVector(b_plot(:,i), servoArms(:,i), 'black') % plot arm positions
            plotVector(servoArms(:,i), P_base_plot(:,i),'green') % plot leg positions
        end
        view(45,10)
end

%% -------------------ANIMATION-----------------

