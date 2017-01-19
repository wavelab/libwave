function plot_costate(fig_index, P, Q)
    figure(fig_index);
    hold on;
    clf;
    
    Pplot = reshape(P, length(Q)^2, length(P));
    plot(T, Pplot(1:8, :)');
    title('Costate');
end

