function R = rotate(theta)
    R = [
        cos(theta), -sin(theta);
        sin(theta), cos(theta);
    ];
endfunction

point = [2.0; 1.0]
t = [1.0; 2.0]

result = rotate(pi) * point + t

% inv(rotate(pi)) * (result - t)
(inv(rotate(pi)) * result) - t
