function varargout = plotez(P,varargin)
% easily plot data stored in an array or a cell of arrays
%
% Joel T Harter
% 2024/5/16

if iscell(P)
    L=cell(size(P));
    oldhold=ishold;
    hold on
    for i=1:numel(P)
        L{i}=plotez(P{i},varargin{:});
    end
    if ~oldhold
        hold off
    end
else
    P=num2cell(P,1);
    n=length(P);% number of dimensions
    if n==3
        L=plot3(P{:},varargin{:});
    else
        L=plot(P{:},varargin{:});
    end
end
if nargout>=1
    varargout={L};
end