function drawbox(fig_index, x, y, h, scale)
    % function drawbox(x,y,h,scale,fig)
    % This function plots a box at position x,y heading h and size scale on
    % figure number fig.


    % car outline
    box = [-1 -0.5; 1 -0.5; 1 0.5; -1 0.5; -1 -0.5];

    % size scaling
    box = scale*box;

    % rotation matrix
    R = [cos(h) -sin(h); sin(h) cos(h)];
    box = (R*box')';

    % centre
    box(:,1) = box(:,1)+x;
    box(:,2) = box(:,2)+y;

    % plot
    figure(fig_index);
    plot(box(:,1), box(:,2), 'b','LineWidth', 2);
    axis equal
end