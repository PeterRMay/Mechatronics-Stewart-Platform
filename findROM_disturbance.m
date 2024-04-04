function [TRange, PhiRange] = findROM_disturbance(platformParams, plotBool, testRange)
arguments
    platformParams
    plotBool = false;
    testRange = linspace(-400,200,50000);
end


ArmLegRatio = platformParams.armlegratio;
PlatformBaseRatio = platformParams.platformbaseratio;
PlatformRadius = platformParams.radius;
RestingLegLength = platformParams.restingleglength;
servoMotorRange = platformParams.servorange;
h0 = platformParams.defaultHeight;

TRange = zeros(3,2);
PhiRange = zeros(3,2);


for q = 1:6
    if q < 4
        currentError = true;
        previousError = true;
        T = zeros(3,1);
        T_commanded = zeros(3,1);
        T_disturbance = zeros(3,1);
        Phi = zeros(3,1);
        Phi_commanded = zeros(3,1);
        Phi_disturbance = zeros(3,1);
        minValue = NaN(1);
        maxValue = NaN(1);

        for i = 1:length(testRange)
            T_disturbance(q) = 0.2.*testRange(i);

            % convert disturbance to platform input
            [T, Phi] = baseToStaticPlatformPosition(T_disturbance,Phi_disturbance,T_commanded,Phi_commanded,h0);
            T(3,1) = T(3,1) - h0;
            currentError = StewartPlatformEqs(T, Phi, ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange);
            if previousError == true && currentError == false % transition from out of range to in range
                
                minValue =  0.2.*testRange(i);
            end
            if previousError == false && currentError == true % transition into range
              
                maxValue =  0.2.*testRange(i);
            end
            previousError = currentError;
        end
        if isnan(minValue)
            error("min bound not exceeded for T(" + q + "), decrease lower bound of testing range")
        elseif isnan(maxValue)
            error("max bound not exceeded for T(" + q + "), increase upper bound of testing range")
        end
        TRange(q,:) = [minValue maxValue];
        

    else
        currentError = true;
        previousError = true;
        T = zeros(3,1);
        T_commanded = zeros(3,1);
        T_disturbance = zeros(3,1);
        Phi = zeros(3,1);
        Phi_commanded = zeros(3,1);
        Phi_disturbance = zeros(3,1);
        minValue = NaN(1);
        maxValue = NaN(1);

        for i = 1:length(testRange)
            Phi_disturbance(q-3) = testRange(i);

            % convert disturbance to platform input
            [T, Phi] = baseToStaticPlatformPosition(T_disturbance,Phi_disturbance,T_commanded,Phi_commanded,h0);
            T(3,1) = T(3,1) - h0;

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
            error("decrease lower bound of testing range for Phi(" + q + ")")
        elseif isnan(maxValue)
            error("increase upper bound of testing range for Phi(" + q + ")")
        end
       PhiRange(q-3,:) = [minValue maxValue];
    end
    
end

if plotBool == true
    TRange
    PhiRange
    subplot(2,2,1)
    StewartPlatformEqs([0 0 TRange(3,2)], [0 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
    title('Max height')
    grid on
    subplot(2,2,2)
    StewartPlatformEqs([0 0 TRange(3,1)], [0 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
    title('Min height')
    grid on
    subplot(2,2,3)
    StewartPlatformEqs([0 0 0], [PhiRange(1,2) 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
    title('Max pitch')
    grid on
    subplot(2,2,4)
    StewartPlatformEqs([0 0 0], [PhiRange(1,1) 0 0], ArmLegRatio, PlatformBaseRatio,RestingLegLength, PlatformRadius, servoMotorRange, true, true);
    title('Min pitch')
    grid on
    sgtitle('Stewart Platform Range Limits')
end
end