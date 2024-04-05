function theta = angleBetweenVectors(u,v)
theta = atan2d(norm(cross(u,v)),dot(u,v));
% theta = acosd(dot(u,v) / (norm(u)*norm(v)));