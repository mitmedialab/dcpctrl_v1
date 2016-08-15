function [ xfiltered ] = lpfiltervec( x, alpha )
%LPFILTERVEC Applies a low-pass filter across a vector (generally not
%useful for real-time applications, only for simulation verifications)

xfiltered = zeros(size(x));
xfiltered(1,:) = x(1,:);

for i=2:size(x,1)
    xfiltered(i,:) = lpfilter(xfiltered(i-1,:),x(i,:),alpha);
end

end

