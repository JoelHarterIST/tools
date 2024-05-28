function u = uvec(v,varargin)
% get a unit vector in the direction of a row vector, column vector, or m-tall stack of n-dimensional row vectors
% 
% INPUT
% v: an [n×1] column vector, [1×n] row vector, or [m×n] array where there are m row vectors of length n
% dim (optional): the vectors' dimension inside the array (e.g. dim=1 means the vectors are column vectors)
%   defaults:
%       1 for column vectors
%       2 for row vectors
%       2 for anything else (assume row vectors)
%
% OUTPUT
% u: an array of the same size as v, where the vectors have all been scaled to unit length
%
% Joel T Harter
% 2024/5/15

if nargin==1||isempty(varargin{1})
    if size(v,2)==1
        dim=1;
    else
        dim=2;
    end
else
    dim=varargin{1};
end

u=v./vecnorm(v,2,dim);