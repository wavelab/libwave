function milestones = prm_sample(nb_samples, map, pos_start, pos_end, map_min, map_max)
	disp('Sample map');
	tic;
	milestones = sample_map(nb_samples,map,map_min,map_max,...
		pos_start(1:2),pos_end);
	plot_omap(1, map, pos_start, pos_end, 0.1);
	plot_milestones(1, milestones);
	drawnow;
	toc;
	disp('');
end
