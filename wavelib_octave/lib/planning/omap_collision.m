function collision = omap_collision(map, x0, y0, x1, y1)
    collision = 0;
	[x, y] = bresenham_omap(x0, y0, x1, y1);

	line = [x, y];
	for i = 1:size(line,1)
		if map(line(i, 1), line(i, 2)) == 1
			collision = 1;
		end
	end
end