% ************************************************************************
% File Name   : cross2d.m (function m-file)
% Author      : Robert TULLY
% e-mail: rtully95@gmail.com 
% Date        : 01/22/2021
% Description : Calculates the cross product of two 2-dimensional vectors.
%               Input : 
%                   a: The first vector.
%                   b: The second vector.
%               Output: 
%                   Returns the cross product of the input vectors.
% ************************************************************************
function result = cross2d(a, b)
    result = a(1,1) * b(1,2) - a(1,2) * b(1,1);
end