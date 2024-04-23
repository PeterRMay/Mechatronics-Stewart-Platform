function errorFlag = StewartPlatformEqs(T,Phi, platformParams, plotBool, displayWarnings)
    arguments
        T
        Phi
        platformParams
        plotBool = false
        displayWarnings = false
    end
    errorFlag = false;
    
    armLegRatio = platformParams.armlegratio;
    platformBaseRatio = platformParams.platformbaseratio;
    legRestingLength = platformParams.restingleglength;
    platformRadius = platformParams.radius;
    servoMotorRange = platformParams.servorange;
    servoOffset = platformParams.servoOffset;
    ballJointRange = platformParams.ballJointRange;
    angleBetweenLegPairs = platformParams.angleBetweenLegPairs;

    % inputs:
    % x, y, z relative displacements into T matrix
    x = T(1);
    y = T(2);
    z = T(3);
    
    % phi, theta, psi
    phi = Phi(1);
    theta = Phi(2);
    psi = Phi(3);

    % leg parameters
    s = sqrt(legRestingLength^2 / (1+armLegRatio)); % length of rod
    a = s * armLegRatio; % length of servo arm
    
    %% constants:
    % b[] distances to each leg, array of 3x1 location vectors
    b1 = [platformRadius;0;0]; % ADD a name for length
    angleFromAxis = angleBetweenLegPairs / 2;
    legAngles = [-angleFromAxis,angleFromAxis,120-angleFromAxis,120+angleFromAxis,240-angleFromAxis,240+angleFromAxis] + 120; %% IMPORTANT: added 120 to match up axes with fwd kinematics
    legAngles = circshift(legAngles,[0 1]);
    b = zeros(3,6);
    for i = 1:6
        b(:,i) = rotz(legAngles(i))*b1;
    end
    
    % p[] distances from origin of platform to the joints
    p = b*platformBaseRatio; % make top platform smaller than bottom
    defaultPlatformRotation = 60;
    for i = 1:6
        p(:,i) = rotz(defaultPlatformRotation) * p(:,i); % rotate top platform
    end
    p = circshift(p,[0 1]); % shift matrix to shift which legs connect to which joints
    
    %% define home location
    beta = zeros(1,6); % angle between servo arm and x axis in the xy plane
    % we add or subtract 90 to place servos arms perpendicular to vector
    % through the center of the platform. This can be adjusted based on
    % construction
    for i = 1:2:6
        beta(i) = legAngles(i) + servoOffset; % note this differs from the document, in the document beta is only for even legs
    end
    for i = 2:2:6
        beta(i) = legAngles(i) - servoOffset;
    end
    
    % we now define the home height, h0, assuming 90 degrees between the arms
    % and legs and no rotation or translational inputs
    l0_squared = s^2+a^2; %default leg length using pythagoras theorem bc right angles
    h0 = sqrt(l0_squared - (p(1,1) - b(1,1))^2 - (p(2,1) - b(2,1))^2) - p(3,1);
    if isreal(h0) ~= true
%         warning("Legs too short given platform dimensions")
        errorFlag = true;
    end
    
    % next we define the home position of the servo motors
    % this is a function purely of the desired default platform location and
    % the leg lengths
    Tdefault = [0;0;h0];
    P_default = Tdefault + p;
    alpha0 = ones(1,6) * servoCalc(P_default(:,1),b(:,1),sqrt(l0_squared),s,a,beta(1));
    
    % set center of range of motion equal to alpha0
    servoCenterOfRange = alpha0(1); % center of range of motion
    
    % calculate maximum ranges of motion
    l_max = s + a;
    % l_min = 
    alpha_max = servoCenterOfRange + servoMotorRange/2;
    alpha_min = servoCenterOfRange - servoMotorRange/2;
    
    %% Calculate outputs
    T = [x;y;z+h0]; % location of platform
    
    % compute rotation matrix based on input angles
    R = rotz(psi)*roty(theta)*rotx(phi);
    
    % compute l_i of each leg
    l = (T + R*p) - b;
    
    % take 2norm of each leg and find unit vectors
    d = size(l);
    l_2norm = zeros(1,d(1,2));
    l_unit = zeros(d);
    for i = 1:d(1,2)
        l_2norm(i) = norm(l(:,i));
        l_unit(:,i) = l(:,i) ./ l_2norm(i);
    end
    
    % limit l values
    for i = 1:6
        if l_2norm(i) > l_max
%             warning("Position not possible, exceeded max leg length")
            errorFlag = true;
            l(:,i) = l_unit(:,i) * l_max;
        end
        % not currently needed, alpha limits set lower bounds on height
    %     if norm(l(:,i)) < l_min 
    %         warning("Position not possible, less than min leg length")
    %         l(:,i) = l_unit(:,i) * l_min;
    %     end
    end
    
    % calculate P_base, end of each joint in base reference frame
    P_base = b + l;
    
    % calculate servo angles
    alpha = zeros(1,6);
    servoArms = zeros(3,6);
    for i = 1:6
        alpha(i) = servoCalc(P_base(:,i),b(:,i),l_2norm(i),s,a,beta(i));
        servoArms(:,i) = b(:,i) + a * rotz(beta(i)) * roty(alpha(i)) * [1;0;0];
    end
    
    % limit servo angles
    for i = 1:6
        if alpha(i) < alpha_min
%             warning("Exceeded minimum servo angle")
            errorFlag = true;
            alpha(i) = alpha_min;
        elseif alpha(i) > alpha_max
%             warning("Exceeded maximum servo angle")
            errorFlag = true;
            alpha(i) = alpha_max;
        end
    end
    
    % re calculate platform position

    %
    
    
    
    if plotBool == true
        %% plotting
        
        % make plotting arrays
        P_base_plot = P_base;
        P_base_plot(:,end+1) = P_base(:,1);
        b_plot = b;
        b_plot(:,end+1) = b_plot(:,1);
        
        plot3(P_base_plot(1,:),P_base_plot(2,:),P_base_plot(3,:),'-') % plot platform
        hold on
        plot3(b_plot(1,:),b_plot(2,:),b_plot(3,:),'-') % plot base
        axis equal
        
        % alpha = ones(1,6) * 0; % for entering manual alpha values
        servoArms = zeros(3,6);
        for i = 1:6
        %     plot3([b(1,i) P_base(1,i)], [b(2,i) P_base(2,i)], [b(3,i) P_base(3,i)]); % plot straight leg lengths
            servoArms(:,i) = b(:,i) + a * rotz(beta(i)) * roty(-alpha(i)) * [1; 0; 0]; % calculate arm positions
            plotVector(b(:,i), servoArms(:,i), 'black') % plot arm positions
            plotVector(servoArms(:,i), P_base(:,i),'green') % plot leg positions
        end        

        %plot servo circle
%         for q = 1:360
%         servoArms1 = b(:,i) + a * rotz(beta(i)) * roty(-alpha(i)+q-1) * [1; 0; 0];
%         servoArms2 = b(:,i) + a * rotz(beta(i)) * roty(-alpha(i)+q) * [1; 0; 0];
%         plotVector(servoArms1,servoArms2,'blue')
%         end
%% plot servo range
%         screen = linspace(alpha_min,alpha_max);
%         for q = 2:length(screen)
%             servoArms1 = b(:,i) + a * rotz(beta(i)) * roty(-screen(q-1)) * [1; 0; 0];
%             servoArms2 = b(:,i) + a * rotz(beta(i)) * roty(-screen(q)) * [1; 0; 0];
%             plotVector(servoArms1,servoArms2,'blue')
%         end


        xlabel('x')
        ylabel('y')
        zlabel('z')
    end

    %% verify model values
            % calculate desired leg lengths
        legLengths = P_base - servoArms;
        legLengthsNorm = zeros(1,6);
        for i = 1:6
           legLengthsNorm(i) = norm(legLengths(:,i));
        end
    if abs(legLengthsNorm(1) -s) > 0.1 && displayWarnings == true
%         warning("legs are changing lengths")
    end
    
    jointAngles = zeros(1,6);
    x_servo = zeros(3,6);
    for i = 1:3
        x_servo(:,i*2-1) = rotz(beta(i*2-1)-90)*[1;0;0];
        x_servo(:,i*2) = rotz(beta(i*2)+90)*[1;0;0];
    end
    for i = 1:6
        jointAngles(i) = 90 - angleBetweenVectors(-x_servo(:,i), legLengths(:,i));
        if abs(jointAngles(i)) > ballJointRange
            errorFlag = true;
        end
    end
end