function [P_base] = forwardKinematics(baseJoints, platformJoints, l_2norm, P0)
    
    average = @(a,b) (a+b)./2;

    d = 0.5*norm(baseJoints(:,1) - baseJoints(:,6)); % half of distance between each pair of base joints
    b = norm(baseJoints(:,2) - baseJoints(:,1)); % calculate length between each pair of odd and even legs
    a = norm(average(platformJoints(:,4),platformJoints(:,3)) - average(platformJoints(:,2),platformJoints(:,1))); %calculate length between platform joints
    
    p = zeros(1,3);%calculate p_i, distance from odd leg to projection of platform
    %joint position on base
    h = zeros(1,3); % height of platform joint
    for i = 1:3
        p(i) = (1/(2*b)) * (b.^2 + l_2norm(2*i-1).^2 - l_2norm(2*i)^2); 
        h(i) = sqrt(l_2norm(2*i-1).^2 - p(i).^2);
    end
    
    var = sqrt(3) / 6;
    
    Xp1 = var*(2*b + d - 3*p(1));
    Yp1 = .5*(d+p(1));
    
    Xp2 = -var*(b+2*d);
    Yp2 = 0.5*(b-2*p(2));
    
    Xp3 = -var*(b-d-3*p(3));
    Yp3 = -0.5*(b+d-p(3));
    
    Xp = [Xp1 Xp2 Xp3;
        Yp1 Yp2 Yp3];

    %% calculate x_T1-3
    % set up cost function
%     X = zeros(1,3);
    testFunction = @(X) f1(a,Xp,X,h);
    

    
    X0 = zeros(1,3);
    
    X0(1) = average(P0(1,2),P0(1,1));
    X0(2) = average(P0(1,4),P0(1,3));
    X0(3) = average(P0(1,5),P0(1,6));
   
    testFunction(X0)

    costFunction = @(X) [f1(a,Xp,X,h); f2(a,Xp,X,h); f3(a,Xp,X,h)];
    [X, Fval] = fsolve(costFunction,X0);
%     cost = 1;
%     for i = 1:400 
%         [X,cost] = fminsearch(costFunction,P0(1,:));
%         cost
%         if abs(cost) > 0.001
%             warning("did not converge")
%         end
%     end
    



    Y = zeros(1,3);
    Y(1) = eq25(X,l_2norm,b);
    Y(2) = eq26(l_2norm,b);
    Y(3) = eq27(X,l_2norm,b);

    Z = zeros(1,3);
    Z(1) = eq29(h,X,Xp);
    Z(2) = eq30(h,X,Xp);
    Z(3) = eq31(h,X,Xp);
    
    P_base = [X;Y;Z];

    % apply newton's method
    % take the jacobian of the column vector of the three equations
    
    % input Xt0, the previous/initial guess, and calculate f1-3


    %F(x) = 0
    %where x is a 3x1 eq


    %----------------Equations--------------------

    function out = f1(a,Xp, Xt, h)
        out = a^2 + 2*Xt(1,1)*Xt(1,2) - ...
            2*Xt(1,1)*(Xp(1,1) + sqrt(3)*(Xp(2,1) - Xp(2,2))) - ...
            2*Xp(1,2)*Xt(1,2) - ((sqrt(3)*Xp(1,1) - Xp(2,1) + Xp(2,2))^2 +...
            (h(1)^2 + h(2)^2) - 4*Xp(1,1)^2 - Xp(1,2)^2) +...
            2*sqrt((h(1)^2 -4*(Xt(1,1)-Xp(1,1))^2) * ...
            (h(2)^2 - (Xt(1,2) - Xp(1,2))^2));
    end
    
    function out = f2(a,Xp,Xt,h)
        out = a^2 - 4*Xt(1,1)*Xt(1,3) - 2*Xt(1,1)*(Xp(1,1) - 3*Xp(1,3) +...
            sqrt(3)*(Xp(2,1)-Xp(2,3))) - 2*Xt(1,3)*(-3*Xp(1,1)+Xp(1,3)+...
            sqrt(3)*(Xp(2,1)-Xp(2,3))) - ((sqrt(3)*(Xp(1,1)+Xp(1,3)) - ...
            Xp(2,1) + Xp(2,3))^2 + (h(1)^2 + h(3)^2) - 4*Xp(1,1)^2 - ...
            4*Xp(1,3)^2) + ...
            2*sqrt((h(1)^2 - 4*(Xt(1,1)-Xp(1,1))^2) *...
            (h(3)^2 - 4*(Xt(1,3) - Xp(1,3))^2));
    end

    function out = f3(a,Xp,Xt,h)
        out = a^2 + 2*Xt(1,2)*Xt(1,3) - 2*Xt(1,3)*(Xp(1,3) + sqrt(3)*...
            (Xp(2,2) - Xp(2,3))) - 2*Xp(1,2)*Xt(1,2) -...
            ((sqrt(3)*Xp(1,3) - Xp(2,2) + Xp(2,3))^2 +...
            (h(2)^2 + h(3)^2) - Xp(1,2)^2 - 4*Xp(1,3)^2) +...
            2*sqrt((h(2)^2 - (Xt(1,2) - Xp(1,2))^2) * ...
            (h(3)^2 - 4*(Xt(1,3) - Xp(1,3))^2));
    end


    function YT1 = eq25(Xt,L,b)
        YT1 = sqrt(3) * Xt(1, 1) +(L(1)^2-L(2)^2)/b;
    end
    function YT2 = eq26(L,b)
        YT2 = (L(4)^2 - L(3)^2)/(2*b);
    end
    function YT3 = eq27(Xt,L,b)
        YT3 = -sqrt(3)*Xt(1,3) + (L(5)^2 - L(6)^2)/b;
    end
    function ZT1 = eq29(h,Xt,Xp)
        ZT1 = sqrt(h(1)^2 - 4*(Xt(1,1) - Xp(1,1))^2);
    end
    function ZT2 = eq30(h,Xt,Xp)
        ZT2 = sqrt(h(2)^2 - ((Xt(1,2) - Xp(1,2))^2));
    end
    function ZT3 = eq31(h,Xt,Xp)
        ZT3 = sqrt(h(3)^2 - 4*(Xt(1,3) - Xp(1,3))^2);
    end



end