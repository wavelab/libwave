function [ colliding, pt ] = EdgeCollision( Edge1, Edge2, tol )
%EDGECOLLISION Checks if two edges are colliding
%   Edge1 -> First edge being checked
%   Edge2 -> Second edge being checked
%   return-> 1: if colliding
%            0: if not colliding
% Edges are represented by endpoints [x1 y1 x2 y2]

if nargin < 3
    tol = 0;
end


p1 = Edge1(1:2);
p2 = Edge1(3:4);
p3 = Edge2(1:2);
p4 = Edge2(3:4);

% If lines are the same
if (abs((p1(1)-p3(1)) <= tol) && (abs(p1(2)-p3(2))<=tol) && (abs(p2(1)-p4(1))<=tol) && (abs(p2(2)-p4(2))<=tol))
  colliding = true;
  pt = p1;
  return;
end

% From Introduction to Algorithms, Cormen et. al.
d = cross([p1-p3,0; p2-p3,0; p3-p1,0; p4-p1,0]',...
          [p4-p3,0; p4-p3,0; p2-p1,0; p2-p1,0]');
d = d(end,:);

colliding = false;
if ((d(1)>0 && d(2)<0) || (d(1)<0 && d(2)>0)) && ((d(3)>0 && d(4)<0) || (d(3)<0 && d(4)>0))
    colliding = true;
elseif (abs(d(1))<100*eps && OnSegment(p3,p4,p1))
    colliding = true;
elseif (abs(d(2))<100*eps && OnSegment(p3,p4,p2))
    colliding = true;
elseif (abs(d(3))<100*eps && OnSegment(p1,p2,p3))
    colliding = true;
elseif (abs(d(4))<100*eps && OnSegment(p1,p2,p4))
    colliding = true;
end
   
% Find intersection point
if (colliding)
    [ e1.A, e1.b ] = EdgePtsToVec( Edge1 );
    [ e2.A, e2.b ] = EdgePtsToVec( Edge2 );
    [pt, colliding] = AffineIntersect( e1, e2 );
else
    pt = [0 0];
end

return

function val = OnSegment(pi,pj,pk)
% OnSegment determines if a point known to be collinear with a segment
% lies on that segment.  Collinearity is established by a zero cross
% product.
% NOTE: this differs from the textbook algorithm in an important way.  The
% inequalities are <, not <=, so that coincident endpoints will not be
% considered collisions, but an endpoint lying in the middle of another
% line segment will be, as happens at corners.
if (min([pi(1),pj(1)]) <= pk(1) && pk(1) <= max([pi(1),pj(1)]))...
   && (min([pi(2),pj(2)]) <= pk(2) && pk(2) <= max([pi(2),pj(2)]))
    val = true;
else
    val = false;
end
