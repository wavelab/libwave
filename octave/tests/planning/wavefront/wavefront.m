% Wavefront example
addpath('../../../lib/planning');


%% Create AVI object
vidObj = VideoWriter('wavefront.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 5;
open(vidObj);

% Vehicle start and end position
startPos = [21.5 18.5];
endPos = [2 3];

%% Set up environment

% Region Bounds
posMinBound = [-2 -2];Kelly Kapowski
posMaxBound = [25 20];

% Number of obstacles
numObsts = 6;
% Size bounds on obstacles
minLen.a = 3;
maxLen.a = 6;
minLen.b = 3;
maxLen.b = 10;

% Random environment generation
obstBuffer = 0.5;
maxCount = 10000;
seedNumber = rand('state');
[aObsts,bObsts,obsPtsStore] = polygonal_world( ...
    posMinBound, ...
    posMaxBound, ...
    minLen, ...
    maxLen, ...
    numObsts, ...
    startPos, ...
    endPos, ...
    obstBuffer, ...
    maxCount ...
);
for i = 1:numObsts
    obsCentroid(i,:) = (obsPtsStore(1,2*(i-1)+1:2*i)+obsPtsStore(3,2*(i-1)+1:2*i))/2;
end

% Plot random environment
figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);
hold off

%% Define grid-based representation
N = 50; % X-axis
M = 50; % Y-axis
map = zeros(N,M);
rx = posMaxBound(1)-posMinBound(1);
ry = posMaxBound(2)-posMinBound(2);

for i = 1:N
    for j = 1:M
        x(i,j) = posMinBound(1) + i/N*rx;
        y(i,j) = posMinBound(2) + j/M*ry;
    end
end
for k=1:numObsts
    c = inpolygon(x,y, obsPtsStore(:,2*(k-1)+1),obsPtsStore(:,2*k));
    map = map + c;
end

figure(2);clf;hold on;
colormap('bone')
imagesc(1-map');

%% Graph setup

% Start node
[startx, starti] = min(abs(x(:,1)-startPos(1)));
[starty, startj] = min(abs(y(1,:)-startPos(2)));
start = (M-1)*starti + startj;

% End node
[finishx, finishi] = min(abs(x(:,1)-endPos(1)));
[finishy, finishj] = min(abs(y(1,:)-endPos(2)));
finish = M*(finishi-1) + finishj;

% Node numbering and graph connectivity
e = sparse(N*M,N*M);

% For each cell in map
for i=1:N
    for j = 1:M
        % If cell is empty
        if (map(i,j) == 0)
            cur = M*(i-1)+j;
            % Link up if empty
            if (i>1)
                if (map(i-1,j) == 0)
                    e(cur, cur-M) = 1;
                    e(cur-M,cur) = 1;
                end
            end
            % Link left
            if (j>1)
                if (map(i,j-1) == 0)
                    e(cur, cur-1) = 1;
                    e(cur-1,cur) = 1;
                end
            end
            % Link down
            if (i<N)
                if (map(i+1,j) == 0)
                    e(cur, cur+M) = 1;
                    e(cur+M,cur) = 1;
                end
            end
            % Link right
            if (j<M)
                if (map(i,j+1) == 0)
                    e(cur, cur+1) = 1;
                    e(cur+1,cur) = 1;
                end
            end
        end % if empty
    end % j
end % i

%% Wavefront
% Essentially a breadth-first search for the cells that can be reached

% Initialize open set (node cost)
O = [finish 0];
% Initialize closed set (same form as open set)
C = [];
done = 0;

dist = 0;
wave = 2*(N+M)*map;
figure(4);
while (~done)
    % Check end condition
    if (length(O)==0)
        done = 1;
        continue;
    end

    % Grab next node in open set
    curnode = O(1,:);
    
    % Move to closed set and save distance for plotting
    C = [C; curnode];
    O = O([2:end],:);
    curi = floor((curnode(1)-1)/M)+1;
    curj = mod(curnode(1),M);
    if (curj==0) curj=M; end
    wave(curi,curj) = curnode(2);

    % Get all neighbours of current node
    neigh = find(e(curnode(1),:)==1);
    
    % Process each neighbour
    for i=1:length(neigh)
        % If neighbour is already in closed list, skip it
        found = find(C(:,1)==neigh(i),1);
        if (length(found)==1)
            continue;
        end
        % If neighbour is already in open list, skip it
        found = find(O(:,1)==neigh(i),1);
        % Otherwise, add to open list at the bottom
        if (length(found)==0)
            O = [O; neigh(i) curnode(2)+1]; 
        end
    end
    if (curnode(2) > dist)
        clf; hold on;
        axis equal;
        axis([0 N 0 M]);
        colormap('default')
        imagesc(wave', [0 1.5*(M+N)])
        plot(finishi,finishj,'r*');
        plot(starti,startj,'b*');
        dist = curnode(2);
        writeVideo(vidObj, getframe(gcf));

    end
end

%% Shortest path

len = wave(starti,startj);
path = zeros(len,2);
path(1,:) = [starti startj];
for i=1:len
    options = [];
    if (path(i,1)>1)
        options = [options; [path(i,1)-1 path(i,2)]];
    end
    if (path(i,1)<N)
        options = [options; [path(i,1)+1 path(i,2)]];
    end
    if (path(i,2)>1)
        options = [options; [path(i,1) path(i,2)-1]];
    end
    if (path(i,2)<M)
        options = [options; [path(i,1) path(i,2)+1]];
    end
    oplen = length(options(:,1));
    for j = 1:oplen
        costs(j) = wave(options(j,1),options(j,2));
    end
    [dist, best] = min(costs);
    path(i+1,:) = options(best,:);
end

figure(4); hold on;
plot(path(:,1),path(:,2), 'rx-')
for i=1:20
    writeVideo(vidObj, getframe(gcf));
end
close(vidObj);