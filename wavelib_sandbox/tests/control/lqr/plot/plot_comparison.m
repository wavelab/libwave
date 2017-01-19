function plot_comparison(fig_index, T, x, x2, x3, u, u2, u3, Ju, Jx, Ju2, Jx2, Ju3, Jx3)
    figure(4);
    hold on;
    clf;

    subplot(5, 1, 1);
    hold on;
    plot(T,x(1,:),'b-');
    plot(T,x2(1,:),'r-');
    plot(T,x3(1,:),'g-');
    title('Angle of Attack')

    subplot(5, 1, 2);
    hold on;
    plot(T,x(2,:),'b-');
    plot(T,x2(2,:),'r-');
    plot(T,x3(2,:),'g-');
    title('Pitch Angle');

    subplot(5, 1, 3);
    hold on;
    plot(T,x(3,:),'b-');
    plot(T,x2(3,:),'r-');
    plot(T,x3(3,:),'g-');
    title('Pitch Rate');

    subplot(5, 1, 4);
    hold on;
    plot(T(1:end-1),u(:),'b-');
    plot(T(1:end-1),u2(:),'r-');
    plot(T(1:end-1),u3(:),'g-');
    title('Deflection Angle');

    subplot(5, 1, 5);
    hold on;
    plot(Ju,Jx,'bx', 'MarkerSize',10,'LineWidth',2)
    plot(Ju2,Jx2,'rx', 'MarkerSize',10,'LineWidth',2)
    plot(Ju3,Jx3,'gx', 'MarkerSize',10,'LineWidth',2)
    title('Cost Comparison, state vs control')
    xlabel('Input costs');
    ylabel('State error costs');
end

