function [m] = inversescannerbres(M,N,x,y,theta,r,rmax)
% Calculates the inverse measurement model for a laser scanner through
% raytracing with Bresenham's algorithm, assigns low probability of object
% on ray, high probability at end.  Returns cells and probabilities.

% Range finder inverse measurement model
x1 = max(1,min(M,round(x)));
y1 = max(1,min(N,round(y)));

endpt = [x y] + r * [cos(theta), sin(theta)];

x2 = max(1, min(M, round(endpt(1))));
y2 = max(1, min(N, round(endpt(2))));

[list(:,1), list(:,2)] = bresenham(x1,y1,x2,y2);
m = [list 0.4*ones(length(list),1)];
if (r<rmax)
    m(end,3) = 0.6;
end


