function val = polygonsOverlap(poly1, poly2)
%POLYGONSOVERLAP Checks if two polygons intersect
%   poly1 - N1x2 matrix of x,y coordinates of polygon vertices
%   poly2 - N2x2 matrix of x,y coordinates of polygon vertices
% ASSUMES:
%   - Polygon vertices are ordered counter clockwise (can be enforced with
%     our scripts)

val = false;

% Simple test to check if 1 is fully or partially enclosed in polygon 2
if sum(inpolygon(poly1(:,1),poly1(:,2),poly2(:,1),poly2(:,2)))
    val = true;
    return
end

% Simple test to check if 2 is fully or partially enclosed in polygon 1
if sum(inpolygon(poly2(:,1),poly2(:,2),poly1(:,1),poly1(:,2)))
    val = true;
    return
end

% Close the polygons
poly1 = [poly1;poly1(1,:)];
obstEdges = [poly2,[poly2(2:end,:);poly2(1,:)]];
% Loop over all possible intersections
for vertex = 1:(length(poly1)-1)
    if (CheckCollision(poly1(vertex,:), poly1(vertex+1,:), obstEdges))
        val = true;
        return
    end
end