function [map, map_min, map_max] = load_omap(image_path)
	I = imread(image_path);
	map = im2bw(I, 0.7);  % Convert to 0 - 1 image
	map = 1 - flipud(map)'; % Convert to 0 free, 1 occupied and flip.
	[M, N] = size(map);  % Map size
	% map = imresize(map, [M * 0.1, N * 0.1]);
	map = im2bw(map, 0.2);  % Convert to 0 - 1 image

	[M, N] = size(map);  % Map size
	map_min = [1 1];
	map_max = [M N];
end