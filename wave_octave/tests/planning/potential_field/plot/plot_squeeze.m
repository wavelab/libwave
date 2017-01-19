function plot_squeeze(subplot_index, t, x, X, Y, gV, obsPtsStore, posMinBound, posMaxBound, startPos, endPos)
    subplot(3, 1, subplot_index); 
    hold on;
    
    plot(x(1, 1:t), x(2, 1:t));
    quiver(X, Y, squeeze(-gV(1, :, :)), squeeze(-gV(2, :, :)));
    plotEnvironment(obsPtsStore, posMinBound, posMaxBound, startPos, endPos);
end