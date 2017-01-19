function [R, t, corr, error, data2] = icp2(data1, data2, R0, t0, res)

% [R, t, corr, error, data2] = icp2(data1, data2, res, tri)
% 
% This is an implementation of the Iterative Closest Point (ICP) algorithm.
% The function takes two data sets and registers data2 with data1. It is
% assumed that R0 and t0 represent an approximate registration. The code
% iterates till no more correspondences can be found.
%
% This is a modified version (12 April, 2005). It is more accurate and has 
% less chances of getting stuck in a local minimum as opposed to my earlier
% version icp.m 
%
% Arguments: data1 - 2 x n matrix of the x and y coordinates of data set 1
%            data2 - 2 x m matrix of the x and y coordinates of data set 2
%            R0 - initial rotation estimate
%            T0 - initial translation estimate
%            res   - the tolerance distance for establishing closest point
%                     correspondences. Normally set equal to the resolution
%                     of data1
% Returns: R - 2 x 2 rotation matrix used to register data2
%          t - 2 x 1 accumulative translation vector used to register data2
%          corr - p x 2 matrix of the index no.s of the corresponding points of
%                 data1 and data2
%          error - the mean error between the corresponding points of data1
%                  and data2 (normalized with res)
%          data2 - 2 x m matrix of the registered data2 
%
% Copyright : This code was modified by Steven Waslander for 2D use.
% 
% Copyright : This code is written by Ajmal Saeed Mian {ajmal.mian@uwa.edu.au}
%              Computer Science, The University of Western Australia. The code
%              may be used, modified and distributed for research purposes with
%              acknowledgement of the author and inclusion of this copyright information.

maxIter1 = 1000;  % Iterations
maxIter2 = 50;  % Iterations
c1 = 0;  % Counters to see if correspondence has stopped improving
c2 = 1;  % Current count
R = R0;
t = t0;

data2 = R*data2; % Rotate data2 scan
data2 = [data2(1,:)+t(1); data2(2,:)+t(2)]; % Transform data2 scan

e1 = 1000001;
e2 = 1000000;
n = 0;
m = 0;

NS1 = KDTreeSearcher(data1','Distance','euclidean');

while n < maxIter1  
    c1 = c2; % Store previous correspondence count
    e1 = e2;
 
    [corr, D] = knnsearch(NS1, data2'); % Search for nearest neighbours using delaunay search
    corr(:,2:3) = [[1 : length(corr)]' D];    % Create correspondence matrix with indexing
    corr(find(D>2*res),:) = [];  % Remove correspondences that exceed 2*tolerance
    
    corr = -sortrows(-corr,3);  % Sort rows by descending distance between points
    corr = sortrows(corr,1);   % Sort rows by data2 index 
    [B, Bi, Bj] = unique(corr(:,1)); % Find points that have been corresponded to the same point
    corr = corr(Bi,:);  % Keep the closest correspondences
    
    [R1, t1] = reg(data1, data2, corr); % Incremental registration using svd
    
    data2 = R1*data2; % Rotate data2 scan
    data2 = [data2(1,:)+t1(1); data2(2,:)+t1(2)]; % Transform data2 scan
    R = R1*R;
    t = R1*t + t1;    
    c2 = length(corr);        
    n = n + 1;
    e2 = sum(corr(:,3))/(length(corr)*res);
    if c2 == c1
        m = m + 1;
        if (abs(e2-e1)<res/10000) || (m > maxIter2)
            break;
        end
    end
    error = min(e1,e2);
end

%-----------------------------------------------------------------
function [R1, t1] = reg(data1, data2, corr)
n = length(corr); 
M = data1(:,corr(:,1)); 
mm = mean(M,2);
S = data2(:,corr(:,2));
ms = mean(S,2); 
Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2)];
Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2)];
K = Sshifted*Mshifted';
K = K/n;
[U A V] = svd(K);
R1 = V*U';
if det(R1)<0
    B = eye(2);
    B(2,2) = det(V*U');
    R1 = V*B*U';
end
R1 = R1./det(R1);
t1 = mm - R1*ms;

% function [R1, t1] = reg2(data1, data2, corr)
% n = length(corr); 
% M = data1(:,corr(:,1)); 
% S = data2(:,corr(:,2));
% for i = 1:10
%    
%     
%     
% end
