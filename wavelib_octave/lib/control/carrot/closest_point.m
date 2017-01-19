function pt = closest_point(point, edge)
	edge = double(edge);
    x1 = edge(1);
    y1 = edge(2);
    x2 = edge(3);
    y2 = edge(4);

    % pre-check
    if x1 == x2
        pt = [x1; point(2)];
        return;
    elseif y1 == y2
        pt = [point(1); y1];
        return;
    end

	% calculate closest point
    p = [point(1); point(2)];
    a = [edge(1); edge(2)];
    b = [edge(3); edge(4)];
	v1 = p - a;
	v2 = b - a;
	t = dot(v1, v2) / (norm(v2))^2;

	if t < 0
		pt = a;
		return;
	elseif t > 1
		pt = b;
		return;
	end

	pt = a + t * v2;
    
end
