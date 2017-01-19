function [minD,minPt, d, pt, ind] = minDistToEdges(v,edges, fig)
% Computes the shortest distance to a sequence of edges, which may form a
% path, for example.  Returns the min distance, the closest point, and the 
% list of distances and closest points for each edge 
if (nargin < 3)
   fig = 0; 
end


n = length(edges(:,1));

for i=1:n
    [d(i), pt(i,:)] = distToEdge(v,edges(i,:));
end

[minD, ind] = min(d);
minPt = pt(ind,:);

if (fig)
    figure(fig); hold on;
    l = [v; minPt];
    plot(l(:,1),l(:,2),'g');
end