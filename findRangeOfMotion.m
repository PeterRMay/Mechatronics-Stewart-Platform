clear all; close all; clc;
%% initialize standard values
stdPlatformBaseRatio = 0.5;
stdArmLegRatio = 0.1495;
stdRestingLegLength = 11/12;

%% test
% platformParams = struct;
% platformParams.armlegratio = stdArmLegRatio;
% platformParams.platformbaseratio = stdPlatformBaseRatio;
% platformParams.radius = 1;
% platformParams.restingleglength = stdRestingLegLength;
% platformParams.servorange = 180;
% [trange,phirange] = findROM(platformParams, true);

%% find optimal armlegratio
clc; clear platformParams
platformParams = struct;
platformParams.armlegratio = stdArmLegRatio;
platformParams.platformbaseratio = stdPlatformBaseRatio;
platformParams.radius = 1;
platformParams.restingleglength = stdRestingLegLength;
platformParams.servorange = 180;

n = 20;
minratio = 0.10;
maxratio = 1;
ratioRange = linspace(minratio, maxratio, n);
% note, arm/leg ratio is related to servo arm length
% by a = ratio*9.4847

tranges = zeros(3,2,n);
phiranges = tranges;
xrange = zeros(1,n);
yrange = xrange;
zrange = xrange;
rollrange = xrange;
pitchrange = xrange;
yawrange = xrange;

inchesPerPlatform = 12;

servoArmLength = zeros(1,n);
LinkageLength = zeros(1,n);

for i = 1:n
    platformParams.armlegratio = ratioRange(i);
    [tranges(:,:,i),phiranges(:,:,i)] = findROM(platformParams);
    xrange(i) = inchesPerPlatform.*(tranges(1,2,i) - tranges(1,1,i));
    yrange(i) = inchesPerPlatform.*(tranges(2,2,i) - tranges(2,1,i));
    zrange(i) = inchesPerPlatform.*(tranges(3,2,i) - tranges(3,1,i));
    rollrange(i) = phiranges(1,2,i) - phiranges(1,1,i);
    pitchrange(i) = phiranges(2,2,i) - phiranges(2,1,i);
    yawrange(i) = phiranges(3,2,i) - phiranges(3,1,i);

    % leg parameters, pulled from stewartplatformeqs
    LinkageLength(i) = sqrt(platformParams.restingleglength^2 / (1+platformParams.armlegratio)); % length of rod
    servoArmLength(i) = inchesPerPlatform.*(LinkageLength(i) * platformParams.armlegratio);
    LinkageLength(i) = inchesPerPlatform.*LinkageLength(i);% length of servo arm
end

Trange = [xrange; rollrange; yrange; pitchrange; zrange; yawrange];
labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
subplot(3,2,i)
plot(servoArmLength,Trange(i,:))
title(labels(i))
xlabel("arm length (in)")
ylabel(units(i))
end
sgtitle("Range of motion for varying arm/leg ratios")


%% find optimal platform size
clc; clear platformParams
platformParams = struct;
platformParams.armlegratio = stdArmLegRatio;
platformParams.platformbaseratio = stdPlatformBaseRatio;
platformParams.radius = 1;
platformParams.restingleglength = stdRestingLegLength;
platformParams.servorange = 180;
n = 20;
minratio = 4/10;
maxratio = 1;
ratioRange = linspace(minratio, maxratio, n);

tranges = zeros(3,2,n);
phiranges = tranges;
xrange = zeros(1,n);
yrange = xrange;
zrange = xrange;
rollrange = xrange;
pitchrange = xrange;
yawrange = xrange;

inchesPerPlatform = 12;

servoArmLength = zeros(1,n);
LinkageLength = zeros(1,n);

for i = 1:n
    platformParams.platformbaseratio = ratioRange(i);
    [tranges(:,:,i),phiranges(:,:,i)] = findROM(platformParams);
    xrange(i) = inchesPerPlatform.*(tranges(1,2,i) - tranges(1,1,i));
    yrange(i) = inchesPerPlatform.*(tranges(2,2,i) - tranges(2,1,i));
    zrange(i) = inchesPerPlatform.*(tranges(3,2,i) - tranges(3,1,i));
    rollrange(i) = phiranges(1,2,i) - phiranges(1,1,i);
    pitchrange(i) = phiranges(2,2,i) - phiranges(2,1,i);
    yawrange(i) = phiranges(3,2,i) - phiranges(3,1,i);

    % leg parameters, pulled from stewartplatformeqs
    LinkageLength(i) = sqrt(platformParams.restingleglength^2 / (1+platformParams.armlegratio)); % length of rod
    servoArmLength(i) = inchesPerPlatform.*(LinkageLength(i) * platformParams.armlegratio);
    LinkageLength(i) = inchesPerPlatform.*LinkageLength(i);% length of servo arm
end

Trange = [xrange; rollrange; yrange; pitchrange; zrange; yawrange];
labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
subplot(3,2,i)
plot(ratioRange.*inchesPerPlatform,Trange(i,:))
title(labels(i))
xlabel("platform size (in)")
ylabel(units(i))
end
sgtitle("Range of motion for varying platform/base ratios")

%% find optimal platform resting height
clear platformParams; clc;
platformParams = struct;
platformParams.armlegratio = stdArmLegRatio;
platformParams.platformbaseratio = stdPlatformBaseRatio;
platformParams.radius = 1;
platformParams.restingleglength = stdRestingLegLength;
platformParams.servorange = 180;
n = 20;
minratio = 0.9;
maxratio = 1.1;
ratioRange = linspace(minratio, maxratio, n);

tranges = zeros(3,2,n);
phiranges = tranges;
xrange = zeros(1,n);
yrange = xrange;
zrange = xrange;
rollrange = xrange;
pitchrange = xrange;
yawrange = xrange;

inchesPerPlatform = 12;

servoArmLength = zeros(1,n);
LinkageLength = zeros(1,n);

for i = 1:n
    platformParams.restingleglength = ratioRange(i);
    [tranges(:,:,i),phiranges(:,:,i)] = findROM(platformParams);
    xrange(i) = inchesPerPlatform.*(tranges(1,2,i) - tranges(1,1,i));
    yrange(i) = inchesPerPlatform.*(tranges(2,2,i) - tranges(2,1,i));
    zrange(i) = inchesPerPlatform.*(tranges(3,2,i) - tranges(3,1,i));
    rollrange(i) = phiranges(1,2,i) - phiranges(1,1,i);
    pitchrange(i) = phiranges(2,2,i) - phiranges(2,1,i);
    yawrange(i) = phiranges(3,2,i) - phiranges(3,1,i);

    % leg parameters, pulled from stewartplatformeqs
    LinkageLength(i) = sqrt(platformParams.restingleglength^2 / (1+platformParams.armlegratio)); % length of rod
    servoArmLength(i) = inchesPerPlatform.*(LinkageLength(i) * platformParams.armlegratio);
    LinkageLength(i) = inchesPerPlatform.*LinkageLength(i);% length of servo arm
end

Trange = [xrange; rollrange; yrange; pitchrange; zrange; yawrange];
labels = ["x", "roll", "y", "pitch", "z", "yaw"];
units = ["in", "deg", "in", "deg","in", "deg",];
figure
for i = 1:6
subplot(3,2,i)
plot(ratioRange.*inchesPerPlatform,Trange(i,:))
title(labels(i))
xlabel("resting leg length (in)")
ylabel(units(i))
end
sgtitle("Range of motion for varying resting leg lengths")
