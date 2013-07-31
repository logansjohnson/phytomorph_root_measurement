function [onEdge] = checkEdge(I)
onEdge = 0;
I(2:end-1,2:end-1) = 0;
% Find blobs
L = bwlabel(I);

if(max(max(L)) > 1)
   onEdge = 1; 
end


end
