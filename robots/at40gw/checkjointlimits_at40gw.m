function [ exceeded, condensed, ltl, gtl ] = checkjointlimits_at40gw( robot, qraw )
% CHECKJOINTLIMITS_AT40GW Checks to see if any joint limits have been
%   exceeded
% INPUTS:
%   qraw - Nx4 
% OUTPUTS:
%   exceeded - Nx4 boolean matrix indicating which movement and which
%       joints have exceeded values
%   condensed - Nx1 boolean vector indicating which movements have any
%       exceeded values
%   lt - Nx4 boolean vector indicating which joints are below a min
%   gt - Nx4 boolean vector indicating which joint are above a max

poslims = [robot.Joint.PosLim];
jmax = [poslims.Max];
jmin = [poslims.Min];

% Hack because J2 is backwards
[jmax(2), jmin(2)] = deal(jmin(2), jmax(2));

ltl = bsxfun(@lt, qraw, jmin);
gtl = bsxfun(@gt, qraw, jmax);

exceeded = ltl | gtl;
condensed = any(exceeded, 2);

end

