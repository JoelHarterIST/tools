function varargout = ploteze(P,varargin)
% shorthand to call plotez.m and set the axes to be equal scale
%
% Joel T Harter
% 2024/5/16

L = plotez(P,varargin{:});
axis equal
if nargout>=1
    varargout={L};
end