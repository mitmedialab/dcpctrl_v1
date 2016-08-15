% COMBINETOOLPATH Quick script to combine output of mm_dometrajgen
% functions into a single cell array, so that they're easier to look at.

%{
    Julian Leland, MIT Media Lab, 2016-07-14
%}

movedata = {};

for n = 1:length(movetypes)
    movedata(n).movetypes = movetypes{n};
    movedata(n).q0raws = q0raws{n};
    movedata(n).k0states = k0states{n};
    movedata(n).k0layers = k0layers{n};
    movedata(n).n0layers = n0layers{n};
    movedata(n).mtypes = mtypes{n};
    movedata(n).nlayers = nlayers{n};
    movedata(n).blayers = blayers{n};
    movedata(n).klayers = klayers{n};
    movedata(n).segts = segts{n};
    movedata(n).absts = absts{n};
    movedata(n).qrawtrajs = qrawtrajs{n};
    movedata(n).qdrawtrajs = qdrawtrajs{n};
    movedata(n).pwmtrajs = pwmtrajs{n};
    movedata(n).ktrajs = ktrajs{n};
    movedata(n).io_ops = io_ops{n};
    movedata(n).delays = delays{n};
end