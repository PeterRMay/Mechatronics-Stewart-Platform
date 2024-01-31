function errorFlag = StewartPlatformEqs(T,Phi)
    
    errorFlag = false;
    
    % inputs:
    % x, y, z relative displacements into T matrix
    x = T(1);
    y = T(2);
    z = T(3);
    
    % phi, theta, psi
    phi = Phi(1);
    theta = Phi(2);
    psi = Phi(3);
    
    % platform parameters
    platformBaseRatio = 0.8;
    
    %% constants:
    % b[] distances to each leg, array of 3x1 location vectors
    % distance to each leg is 1
    b1 = [1;0;0]; % ADD a name for length
    legAngles = [-5,5,115,125,235,245];
    b = zeros(3,6);
    for i = 1:6
        b(:,i) = rotz(legAngles(:,i))*b1;
    end
    
    % p[] distances from origin of platform to the joints
    p = b*platformBaseRatio; % make top platform smaller than bottom
    defaultPlatformRotation = 60;
    for i = 1:6
        p(:,i) = rotz(defaultPlatformRotation) * p(:,i); % rotate top platform
    end
    p = circshift(p,[0 1]); % shift matrix to shift which legs connect to which joints
    
    % leg and arm dimensions
    % minTotalLegLength = norm(P_base(:,1) - b(:,1));
    s = 1; % length of rod
    a = 0.2; % length of servo arm
    
    % servo specs
    servoRange = 180; % servo motor range of motion
    
    % define home location
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
    alpha_max = servoCenterOfRange + servoRange/2;
    alpha_min = servoCenterOfRange - servoRange/2;
    
    %% Calculate outputs
    T = [x;y;z+h0]; % location of platform
    
    % compute rotation matrix based on input angles
    R = rotz(phi)*roty(theta)*rotx(psi);
    
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
        if norm(l(:,i)) > l_max
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
    
    
    
% % %     %% plotting
% % %     
% % %     % make plotting arrays
% % %     P_base_plot = P_base;
% % %     P_base_plot(:,end+1) = P_base(:,1);
% % %     b_plot = b;
% % %     b_plot(:,end+1) = b_plot(:,1);
% % %     
% % %     plot3(P_base_plot(1,:),P_base_plot(2,:),P_base_plot(3,:),'-') % plot platform
% % %     hold on
% % %     plot3(b_plot(1,:),b_plot(2,:),b_plot(3,:),'-') % plot base
% % %     axis equal
% % %     
% % %     % alpha = ones(1,6) * 0; % for entering manual alpha values
% % %     servoArms = zeros(3,6);
% % %     for i = 1:6
% % %     %     plot3([b(1,i) P_base(1,i)], [b(2,i) P_base(2,i)], [b(3,i) P_base(3,i)]); % plot straight leg lengths
% % %         servoArms(:,i) = b(:,i) + a * rotz(beta(i)) * roty(-alpha(i)) * [1; 0; 0]; % calculate arm positions
% % %         plotVector(b(:,i), servoArms(:,i), 'red') % plot arm positions
% % %         plotVector(servoArms(:,i), P_base(:,i),'green') % plot leg positions
% % %     end
% % %     
% % %     % calculate desired leg lengths
% % %     legLengths = P_base - servoArms;
% % %     legLengthsNorm = zeros(1,6);
% % %     for i = 1:6
% % %        legLengthsNorm(i) = norm(legLengths(:,i));
% % %     end
% % % 

end