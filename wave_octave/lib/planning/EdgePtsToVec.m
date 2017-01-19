function [ A, b ] = EdgePtsToVec( edge )
%EDGEPTSTOVEC Converts an edge from point notation to vector notation
%   edge -> Edge represented by endpoints [x1 y1 x2 y2]
%   returns: Line in vector form Ax = b

f = [edge(1); edge(2); 0 ] ;
g = [edge(3); edge(4); 0 ] ;

A = cross(f-g, [0;0;1]) ;
A = A' / norm(A) ;
b = A*f ;
A = A(1:2) ;

