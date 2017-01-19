function [d,pt] = distToEdge(v,edge)
% Computes the shortest distance to a line segment, returns distance and
% closest point

S = edge(3:4) - edge(1:2);
S1 = v - edge(1:2);
m1 = S1*S';
if (m1 <= 0 )
    d =  norm(v-edge(1:2));
    pt = edge(1:2);
else
    m2 = S*S';
    if ( m2 <= m1 )
        d = norm(v-edge(3:4));
        pt = edge(3:4);
    else
        b = m1 / m2;
        pt = edge(1:2) + b*S;
        d = norm(v-pt);
    end
end


