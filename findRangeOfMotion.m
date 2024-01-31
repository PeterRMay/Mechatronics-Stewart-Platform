clear all; close all; clc;

ArmLegRatio = 1/10;
PlatformBaseRatio = 0.5;
PlatformRadius = 1;
RestingLegLength = 1;
TRange = zeros(3,2);
PhiRange = zeros(3,2);
servoMotorRange = 180;

testRange = linspace(-40,40,1000);

for q = 1:6
    if q <4
        currentError = true;
        previousError = true;
        T = zeros(3,1);
        Phi = zeros(3,1);
        minValue = NaN(1);
        maxValue = NaN(1);

        for i = 1:length(testRange)
            T(q) = testRange(i);
            currentError = StewartPlatformEqs(T, Phi, ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange);
            if previousError == true && currentError == false % transition from out of range to in range
                
                minValue = testRange(i);
            end
            if previousError == false && currentError == true % transition into range
              
                maxValue = testRange(i);
            end
            previousError = currentError;
        end
        if isnan(minValue)
            error("min bound not exceeded, decrease lower bound of testing range")
%         elseif isnan(maxValue)
%             error("max bound not exceeded, increase upper bound of testing range")
        end
        TRange(q,:) = [minValue maxValue];
        

    else
        currentError = true;
        previousError = true;
        T = zeros(3,1);
        Phi = zeros(3,1);
        minValue = NaN(1);
        maxValue = NaN(1);
        for i = 1:length(testRange)
            Phi(q-3) = testRange(i);
            currentError = StewartPlatformEqs(T, Phi, ArmLegRatio, PlatformBaseRatio, RestingLegLength, PlatformRadius, servoMotorRange);
            if previousError == true && currentError == false % transition from out of range to in range
                minValue = testRange(i);
            end
            if previousError == false && currentError == true % transition into range
                maxValue = testRange(i);
            end
            previousError = currentError;
        end
        if isnan(minValue)
            error("decrease lower bound of testing range")
%         elseif isnan(maxValue)
%             error("increase upper bound of testing range")
        end
       PhiRange(q-3,:) = [minValue maxValue];
    end
    
end

TRange
PhiRange
subplot(2,2,1)
StewartPlatformEqs([0 0 TRange(3,2)], [0 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
title('Max height')
subplot(2,2,2)
StewartPlatformEqs([0 0 TRange(3,1)], [0 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
title('Min height')
subplot(2,2,3)
StewartPlatformEqs([0 0 0], [PhiRange(1,2) 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
title('Max pitch')
subplot(2,2,4)
StewartPlatformEqs([0 0 0], [PhiRange(1,1) 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
title('Min pitch')