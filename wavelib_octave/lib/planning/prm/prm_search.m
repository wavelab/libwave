function [spath, sdist] = prm_search(map, milestones, nb_edges, start, goal)
	disp('Search graph');
    
	tic;
	edges = connect_edges_omap(map, milestones, nb_edges);
	[spath, sdist] = astar(milestones, edges, start, goal);
	toc;
    
	disp('');
end
