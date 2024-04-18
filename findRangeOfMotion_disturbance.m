clear all; close all; clc;

%% initialize standard values
stdPlatformBaseRatio = 1;
stdArmLegRatio = 0.15;
stdRestingLegLength = 11;
stdRadius = 7;
stdServoRange = 180;
stdServoOffset = 100;
stdBallJointRange = 25/2;
stdAngleBetweenLegPairs = 60;

a = 2;

stdPlatformParams = struct;
stdPlatformParams.armlegratio = stdArmLegRatio;
stdPlatformParams.platformbaseratio = stdPlatformBaseRatio;
stdPlatformParams.radius = stdRadius;
stdPlatformParams.restingleglength = stdRestingLegLength;
stdPlatformParams.servorange = stdServoRange;
stdPlatformParams.servoOffset = stdServoOffset;
stdPlatformParams.ballJointRange = stdBallJointRange;
stdPlatformParams.angleBetweenLegPairs = stdAngleBetweenLegPairs;
stdPlatformParams.defaultHeight = findh0(stdPlatformParams);

%% test case
% [Trange, Phirange] = findROM_disturbance(platformParams, true, linspace(-40,40,6500));
platformParams = stdPlatformParams;
[Trange, Phirange] = findROM_disturbance(platformParams, true);
figure
jointAngles = StewartPlatformEqs_JointAngles([0;0;0],[0;0;0],platformParams,true);
jointAngles

%% vary platform to base ratio
clear platformParams;
% close all;

% define testing range
n = 20;
minratio = 0.5;
maxratio = 1;
ratioRange = linspace(minratio, maxratio, n);


% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams.platformbaseratio = ratioRange(i);
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

RangesPlatformBaseRatio = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(ratioRange,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("platform/base ratio")
    ylabel(units(i))
end
sgtitle("Range of motion for varying platform/base ratios")

%% vary arm to leg ratio
clear platformParams;
% close all;

% define testing range
n = 20;
minratio = 1/10;
maxratio = stdArmLegRatio +0.5;
ratioRange = linspace(minratio, maxratio, n);


% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams.armlegratio = ratioRange(i);
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

[RangesPlatformBaseRatio, servoArmLength, LinkageLength] = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(servoArmLength,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("arm length (in)")
    ylabel(units(i))
end
sgtitle("Range of motion for varying arm/leg ratios")

figure
for i = 1:6
    subplot(3,2,i)
    plot(LinkageLength,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("arm length (in)")
    ylabel(units(i))
end
sgtitle("Range of motion for varying arm/leg ratios")

%% vary platform radius
clear platformParams;
% close all;

% define testing range
n = 20;
minratio = 3;
maxratio = 12;
ratioRange = linspace(minratio, maxratio, n);


% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams.radius = ratioRange(i);
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

RangesPlatformBaseRatio = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(ratioRange,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("base radius")
    ylabel(units(i))
end
sgtitle("Range of motion for varying base radius")


%% vary servo offset
clear platformParams;
% close all;

% define testing range
n = 50;
minratio = 80;
maxratio = 130;
ratioRange = linspace(minratio, maxratio, n);


% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams.servoOffset = ratioRange(i); %% change the struct element here
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

RangesPlatformBaseRatio = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(ratioRange,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("servo offset angle")
    ylabel(units(i))
end
sgtitle("Range of motion for varying servo offset angles")

%% vary angle between servos
clear platformParams;
% close all;

% define testing range
n = 20;
minratio = 10;
maxratio = 60;
ratioRange = linspace(minratio, maxratio, n);

% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams.angleBetweenLegPairs = ratioRange(i); %% change the struct element here
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

RangesPlatformBaseRatio = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(ratioRange,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("angle between leg pairs")
    ylabel(units(i))
end
sgtitle("Range of motion for varying angles between leg pairs")

%% %% vary resting leg length
clear platformParams;
% close all;

% define testing range
n = 20;
minratio = 20;
maxratio = 6;
ratioRange = linspace(minratio, maxratio, n);

% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams.restingleglength = ratioRange(i); %% change the struct element here
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

RangesPlatformBaseRatio = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(ratioRange,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("angle between leg pairs")
    ylabel(units(i))
end
sgtitle("Range of motion for varying default leg length")

%% vary leg length
clear platformParams;
% close all;

% define testing range
n = 20;

a = 2;
srange = linspace(5,20,n);
armLegRatioRange = a./srange;
restingLegLengthRange = sqrt(s.^2*(1+stdArmLegRatio));

% create array of platformparam structs
platformParamsArray = cell(1,n);
for i = 1:n 
    platformParams = stdPlatformParams;
    platformParams = armLegRatioRange(i);
    platformParams.restingleglength = restingLegLengthRange(i); %% change the struct element here
    platformParams.defaultHeight = findh0(platformParams);
    platformParamsArray{1,i} = platformParams;
end

RangesPlatformBaseRatio = findRanges(platformParamsArray,n);

labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
    subplot(3,2,i)
    plot(srange,RangesPlatformBaseRatio(i,:))
    title(labels(i))
    xlabel("angle between leg pairs")
    ylabel(units(i))
end
sgtitle("Range of motion for varying leg length")


%%


function [Ranges, servoArmLength, LinkageLength] = findRanges(platformParamsArray, n)
    % initialize arrays
    tranges = zeros(3,2,n);
    phiranges = tranges;
    xrange = zeros(1,n);
    yrange = xrange;
    zrange = xrange;
    rollrange = xrange;
    pitchrange = xrange;
    yawrange = xrange;
    servoArmLength = zeros(1,n);
    LinkageLength = zeros(1,n);

    for i = 1:n
        platformParams = platformParamsArray{1,i};
        [tranges(:,:,i),phiranges(:,:,i)] = findROM_disturbance(platformParams);
        xrange(i) = (tranges(1,2,i) - tranges(1,1,i));
        yrange(i) = (tranges(2,2,i) - tranges(2,1,i));
        zrange(i) = (tranges(3,2,i) - tranges(3,1,i));
        rollrange(i) = phiranges(1,2,i) - phiranges(1,1,i);
        pitchrange(i) = phiranges(2,2,i) - phiranges(2,1,i);
        yawrange(i) = phiranges(3,2,i) - phiranges(3,1,i);
    
        % leg parameters, pulled from stewartplatformeqs
        LinkageLength(i) = sqrt(platformParams.restingleglength^2 / (1+platformParams.armlegratio)); % length of rod
        servoArmLength(i) = (LinkageLength(i) * platformParams.armlegratio);
        LinkageLength(i) = LinkageLength(i);% length of servo arm
    end

    Ranges = [xrange; rollrange; yrange; pitchrange; zrange; yawrange];
end



