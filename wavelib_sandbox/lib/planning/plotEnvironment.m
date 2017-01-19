%takes in the points of the rectangles and plots the environment

function plotEnvironment(ptsStore,posMinBound, posMaxBound, startPos, endPos)

hold on
for i = 1:2:size(ptsStore,2)
    patch(ptsStore(:,i),ptsStore(:,i+1),[1 0 0],'linestyle','-','EdgeColor',[1 0 0], 'LineWidth',2)
end
plot(startPos(1),startPos(2),'b*')
plot(endPos(1),endPos(2),'g*')
patch([posMinBound(1) posMaxBound(1) posMaxBound(1) posMinBound(1)],[posMinBound(2) posMinBound(2) posMaxBound(2) posMaxBound(2)],...
    [1 0 0],'facecolor','none','linestyle','-','EdgeColor',[0 1 1])
hold off
axis equal
axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2)]);