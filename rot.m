function R = rot(varargin)
%(theta, axes, use_deg)
% function to make an elementary 3D rotation matrix from rotation angles and specified rotation axes (x,y, or z)
% use_deg is a flag that can be set to TRUE to use degrees. Default is
% FALSE which means radians will be used.
%
% INPUT
% theta: rotation angle
%   this can be any of the following:
%   *   a single angle for elementary rotations about principle axes
%   *   an n-long array where each entry is a rotation angle
%   *   n separate inputs, each one specifying a rotation angle (these must be the first n input arguments)
%   angles can be in degrees or radians, specified by the use_deg flag; default radians (use_deg=false)
% axes (for 3D rotation matrices): a char containing the rotation axis or axes
%   for n input angles, there should be n axes
%   axes are specified as lowercase letters of axes (e.g. 'xyz')
% use_deg(optional): a logical flag for whether to use degrees
%   *   false (default): means all input rotation angles were given in radians
%   *   true: all input rotation angles were given in degrees
%
% OUTPUT
% R: a rotation matrix
%   *   for a single rotation angle and no axis, this returns a 2×2 rotation matrix
%   *   for a single rotation angle and axis pair, this returns an elementary 3D rotation matrix (i.e. about a principal axis)
%   *   for multiple angle-axis pairs, the individual rotation matrices are calculated and multiplied together to form a single 3×3 rotation matrix
%
% EXAMPLES
% rot(pi/4) = rot(pi/4,false) = rot(45,true) =
%     [ 0.7071   -0.7071
%       0.7071    0.7071 ]
% rot(pi/4,'z') = rot(45,'z',true) =
%     [ 0.7071   -0.7071    0
%       0.7071    0.7071    0
%       0         0         1 ]
% rot([pi/3;pi/4],'xz') = rot(60,45,'xz',true) = rot(pi/3,'x')*rot(pi/4,'z') =
%     [ 0.7071   -0.7071         0
%       0.3536    0.3536   -0.8660
%       0.6124    0.6124    0.5000 ]
%
% AUTHOR
% Joel T Harter
% 2024 Jan 20

% set default/empty input parameters
theta = [];
axes = '';
use_deg = false;% default to use radians
% parse input
for i=1:nargin
    if isnumeric(varargin{i})% interpret any numerical input as a rotation angle and add it to the list of rotation angles
        theta=[theta;varargin{i}(:)];
    elseif ischar(varargin{i})% interpret any char input as a rotation axis and add it to the list of rotation axes
        axes=[axes,varargin{i}];
    elseif islogical(varargin{i})% interpret any logical as the use_deg flag and write it over the previous value
        use_deg=varargin{i};
    end
end

% set default mode to radians
% if nargin < 3% if user doesn't say to use degrees
%     use_deg = false;% Do not use degrees. Use radians.
% end

if length(theta)==1
    if isempty(axes)% no rotation axis provided. return 2D rotation matrix
        % calculate sine and cosine
        if use_deg% calculate using degrees
            c = cosd(theta);
            s = sind(theta);
        else% calculate using radians (default)
            c = cos(theta);
            s = sin(theta);
        end
        R = [c , -s; s, c];% 2D rotation matrix by angle of theta
    else% one rotation angle and axis provided. return elementary 3D rotation
        axisnumber=lower(axes)-119;
        R = eye(3);% start off with I3 identity matrix
        otheraxes = mod(axisnumber + [0,1], 3) + 1;% list next 2 axes in order after chosen rotation axis
        R(otheraxes, otheraxes) = rot(theta,[],use_deg);% replace off-axis entries with 2D rotation matrix
    end
else% multiple angle-axis pairs provided return 3D rotation matrix that represents rotating through each angle-axis pair
    R=eye(3);
    for i=1:length(theta)
        R=R*rot(theta(i),axes(i),use_deg);
    end
end
