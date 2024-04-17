clc
close
T = [0;0;-0.8571];
Phi = [0;0;0];
ArmLegRatio = 3/10;
PlatformBaseRatio = 0.5;
RestingLegLength = 1;
PlatformRadius = 1;
servoMotorRange = 180;
set(groot,'DefaultLineLineWidth',1.5)
StewartPlatformEqs(T,Phi, ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true)

%% range of motion test
stdPlatformBaseRatio = 0.5;
stdArmLegRatio = 0.1495;
stdRestingLegLength = 11/12;

platformParams = struct;
platformParams.armlegratio = stdArmLegRatio;
platformParams.platformbaseratio = stdPlatformBaseRatio;
platformParams.radius = 1;
platformParams.restingleglength = stdRestingLegLength;
platformParams.servorange = 180;

[tRange, phiRange] = findROM(platformParams);
ArmLegRatio = platformParams.armlegratio;
PlatformBaseRatio = platformParams.platformbaseratio;
PlatformRadius = platformParams.radius;
RestingLegLength = platformParams.restingleglength;
servoMotorRange = platformParams.servorange;
T = [0;0;0];

alphaMat = zeros(3,2,6);
for i = 1:3
    for j = 1:2
        Phi = zeros(3,2);
        Phi(i) = phiRange(i,j);
        alphaMat(i,j,:) = StewartPlatformEqs_alpha(T, Phi, ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange);
    end
end

platformRotationRange = phiRange(:,2) - phiRange(:,1);
servoRotationRange = zeros(3,6);
maximumServoRotation = zeros(3,1);
for i = 1:3
    servoRotationRange(i,:) = squeeze(abs(alphaMat(i,2,:) - alphaMat(i,1,:)));
    maximumServoRotation(i) = max(servoRotationRange(i));
end
servoSpeed = linspace(100,400); % deg servo /s
platformRotationPerServoRotation = platformRotationRange ./ maximumServoRotation; % deg servo / deg platform
platformRotationalSpeed = zeros(3,length(servoSpeed));
for i = 1:length(servoSpeed)
    platformRotationalSpeed(:,i) = platformRotationPerServoRotation .* servoSpeed(i);
end
labels = ["phi", "theta", "psi"];
close all;

feetechServoSpeed = 1/(.16/60); % 375 deg/s
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(servoSpeed,platformRotationalSpeed(i,:))
    ylabel(labels(i) + " (deg/s)")
    xlabel("servo speed (deg/s)")
    plot([feetechServoSpeed feetechServoSpeed], [min(platformRotationalSpeed(i,:)), max(platformRotationalSpeed(i,:))])
end


%% forward kinematics test
clear all; clc;
%platform position
P_base = [0.458861149080837	0.338094609392560	-0.796955758473396	-0.796955758473396	0.338094609392560	0.458861149080837;
0.655321635431194	0.725046229629320	0.0697245941981265	-0.0697245941981265	-0.725046229629320	-0.655321635431194;
0.553752646657586	0.553752646657586	0.553752646657586	0.553752646657586	0.553752646657586	0.553752646657586];
%base position
B = [0.996194698091746	-0.422618261740699	-0.573576436351046	-0.573576436351046	-0.422618261740699	0.996194698091746;
0.0871557427476582	0.906307787036650	0.819152044288992	-0.819152044288992	-0.906307787036650	-0.0871557427476582;
0	0	0	0	0	0];

legLengths = [0.958218043131008	0.958218043131008	0.958218043131008	0.958218043131008	0.958218043131008	0.958218043131008];

P_base_0 = P_base;

P_base = forwardKinematics(B,P_base,legLengths,P_base_0)



%%
% % tau = 0.102;
% % s = tf('s');
% % servo = 1 / (tau*s + 1);
% % step(servo)


% % % %% simulink setup
% % % %---------------initialize system constants-----------------
%platform position
P = [0.458861149080837	0.458861149080837	0.338094609392560	-0.796955758473396	-0.796955758473396	0.338094609392560;
-0.655321635431194	0.655321635431194	0.725046229629320	0.0697245941981265	-0.0697245941981265	-0.725046229629320;
0	0	0	0	0	0];
%base position
B = [0.996194698091746	0.996194698091746	-0.422618261740699	-0.573576436351046	-0.573576436351046	-0.422618261740699;
-0.0871557427476582	0.0871557427476582	0.906307787036650	0.819152044288992	-0.819152044288992	-0.906307787036650;
0	0	0	0	0	0];

h0 = 0.5576; %default height
a = 0.0953462589245592; %servo arm length
s = 0.953462589245592; %fixed leg length

%define beta
legAngles = [-5,5,115,125,235,245];
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

%--------------define input signals------------
TF = 200;
n = 50;
t = linspace(0,TF,n);

nullSignal = zeros(3,n);
nullTimeseries = timeseries(nullSignal,t);
onesSignal = ones(3,n);

D = nullTimeseries;
gammaSignal = onesSignal .* [0;10;10];
Gamma = timeseries(gammaSignal,t);
T = nullTimeseries;
Phi = nullTimeseries;


SimOut = sim("StewartPlatformSimulation.slx");

