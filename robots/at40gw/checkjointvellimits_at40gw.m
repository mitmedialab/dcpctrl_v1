function [ exceeded, condensed, ltl, gtl ] = checkjointvellimits_at40gw( robot, qdraw )
%CHECKJOINTVELLIMITS_AT40GW Check if raw velocity limits of joints of
%AT40GW have been exceeded.

%{
    [ exceeded, condensed, ltl, gtl ] = checkjointvellimits_at40gw( robot, qraw )
    Julian Leland, MIT Media Lab, 2016-07-02
    This is Veevee's code (from checkjointlimits_at40gw) that I'm
    shamelessly ripping off.
    
%}
%%
velmap = [robot.Joint.VelMap];
vmax = [velmap.PosKnee];
vmin = [velmap.NegKnee];
%%
ltl = bsxfun(@lt, qdraw, vmin);
gtl = bsxfun(@gt, qdraw, vmax);
%%
exceeded = ltl | gtl;
condensed = any(exceeded, 2);


end

