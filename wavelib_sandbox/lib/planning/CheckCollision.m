function [ inCollision, edge ] = CheckCollision( ptA, ptB, obstEdges )
%CHECKCOLLISION Checks if a line interesects with a set of edges
%   Detailed explanation goes here

% Check for each edge
edge = [];
for k = 1:size(obstEdges,1)
    % If both vertices aren't to the left, right, above, or below the
    % edge's vertices in question, then test for collision
    if ~((max([ptA(1),ptB(1)])<min([obstEdges(k,1),obstEdges(k,3)])) || ...
         (min([ptA(1),ptB(1)])>max([obstEdges(k,1),obstEdges(k,3)])) || ...
         (max([ptA(2),ptB(2)])<min([obstEdges(k,2),obstEdges(k,4)])) || ...
         (min([ptA(2),ptB(2)])>max([obstEdges(k,2),obstEdges(k,4)])))
        if (EdgeCollision([ptA, ptB], obstEdges(k,:)))
            % Eliminate end-to-end contacts from collisions list
            if (sum(abs(ptA-obstEdges(k,1:2)))>0 && ...
                sum(abs(ptB-obstEdges(k,1:2)))>0 && ...
                sum(abs(ptA-obstEdges(k,3:4)))>0 && ...
                sum(abs(ptB-obstEdges(k,3:4)))>0)
            
                edge = k;
                inCollision = 1 ; % In Collision
                return
            end
        end
    end
end
inCollision = 0 ; % Not in collision