function [P_base] = find6P(P,d)
P1 = P(:,1);
P2 = P(:,2);
P3 = P(:,3);

% find unit vectors in direction of new points
unitVectors = zeros(3,3);
interVector = zeros(3,1);
normalize = @(vector) vector / norm(vector);

interVector = P2-P3;
unitVectors(:,1) = normalize(interVector);
interVector = P3-P1;
unitVectors(:,2) = normalize(interVector);
interVector = P1-P2;
unitVectors(:,3) = normalize(interVector);

% step by d in positive and negative direction of unit vectors
P_base = zeros(3,6);
for i = 1:3
    P_base(:,2*i-1) = P(:,i) + unitVectors(:,i) .* d;
    P_base(:,2*i) =  P(:,i) - unitVectors(:,i) .* d;
end
P_base = circshift(P_base, [0 -1]);

% % average3 = @(x) (x(1) + x(2) + x(3))/3;
% % % find center of platform in base reference frame
% % center_x = average3(P(1,:));
% % center_y = average3(P(2,:));
% % center_z = average3(P(3,:));
% % 
% % center = [center_x;center_y;center_z];
% % 
% % % put P and center in the platform reference frame
% % center_platform = center - center;
% % P_platform = R_BaseToPlatform
% % 
% % % find the null() of P(1:2,i) - center(1:2)
% % % center(1:2) should just be zeros
% % % should result in just one vector
% % % this vector is in the direction of the points that we want to create

% rotate all points back to base frame

