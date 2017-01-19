function [ pt, doesIntersect ] = AffineIntersect( edge1, edge2 )
%AFFINEINTERSECT Finds interesection between two lines
%   Params:
%       edge1 ->  First edge to be tested (struct form edge1.A, edge1.b)
%       edge2 -> Second edge to be tested (struct form edge2.A, edge2.b)
%   Returns:
%       pt    -> The interesection point (of form [x y])
%       doesIntersect -> Returns 1 if the points do in fact intersect

A = [edge1.A ; edge2.A] ;
b = [edge1.b ; edge2.b] ;

A = min(A,1E6);

if (cond(A) > 1E6)
    doesIntersect = 0 ;
    pt = [0 0] ;
    return ;
end


pt = (inv(A) * b)' ;
doesIntersect = 1 ;