function [CS, tipCoords] = completeSkeleton(BI, CS, CLIP, WINDOWSIZE)
% completeSkeleton takes a clean skeleton and completes the approximate
%   midline by drawing to the center pixel on the image edge intersecting
%   side and then by finding the root tip on the tip end and drawing to
%   that.
%
% BI : Binarized image of root
% CS : Clean skeleton of root
% CLIP : Constant used for tip finding
% WINDOWSIZE : Window size for finding tip
%
% CS : Resulting complete skeleton
% tipCoords : 1 by 2 vector of xy coordinates of the tip


% Find the edge pixels on blob
BItmp = BI;
BItmp(2:end-1,2:end-1) = 0;
L = bwlabel(BItmp,8);
R = regionprops(L,'Area','PixelIdxList','Centroid');
[big,bigIdx] = max([R.Area]);
BItmp = zeros(size(BItmp));
BItmp(R(bigIdx).PixelIdxList) = 1;
centroidCoords = [R(bigIdx).Centroid(2) R(bigIdx).Centroid(1)];
% Find closest actual pixel to centroid, (estimate)
smallDist = 10000000;
smallDistIdx = 0;
for i = 1:size(R(bigIdx).PixelIdxList)
    tCoords = getCoords(R(bigIdx).PixelIdxList(i),size(BItmp));
    thisDist = ((abs(tCoords(1) - centroidCoords(1)))^2) + ((abs(tCoords(2) - centroidCoords(2)))^2)^.5;
    if(thisDist < smallDist)
       smallDist = thisDist;
       smallDistIdx = i;
    end
end
centroidCoords = getCoords(R(bigIdx).PixelIdxList(smallDistIdx),size(BItmp));

CSorig= CS;
SZ=size(CS);

% Find skeleton ends
top = zeros(1,size(CS,2) + 2);
sides = zeros(size(CS,1),1);
CS = [sides, CS, sides];
CS = [top; CS; top];
N = im2col(CS, [3 3], 'Sliding');
sumN = sum(N,1);
sumN = reshape((sumN),SZ(1),SZ(2));
ends = find( (and(sumN == 2, CSorig == 1)) );
CS = CSorig;
smallDist = 10000000;
smallDistIdx = 0;
for i = 1:size(ends)
    tCoords = getCoords(ends(i),size(CS));
    thisDist = ((abs(tCoords(1) - centroidCoords(1)))^2) + ((abs(tCoords(2) - centroidCoords(2)))^2)^.5;
    if(thisDist < smallDist)
       smallDist = thisDist;
       smallDistIdx = i;
    end
end
endCoords = getCoords(ends(smallDistIdx),size(CS));

% Complete root to edge
[lineidx] = drawline(centroidCoords,endCoords,SZ);

% Now locate tip end of skeleton
tipEndCoord = getCoords(ends(  (~(smallDistIdx-1))+1),size(CS));
ul = [(tipEndCoord(1)-WINDOWSIZE) (tipEndCoord(2)-WINDOWSIZE)];
br = [(tipEndCoord(1)+WINDOWSIZE) (tipEndCoord(2)+WINDOWSIZE)];


% Adjust window to not go past edge
fidx = find(ul<1);
if(numel(fidx)>0)
   ul(fidx) = 1; 
end
if(ul(1) > size(CS,1))
    ul(1) = size(CS,1);
end
if(ul(2) > size(CS,2))
    ul(2) = size(CS,2);
end
fidx = find(br<1);
if(numel(fidx)>0)
   br(fidx) = 1; 
end
if(br(1) > size(CS,1))
    br(1) = size(CS,1);
end
if(br(2) > size(CS,2))
    br(2) = size(CS,2);
end
thisSamp = BI(ul(1):br(1),ul(2):br(2));  


% Find tip
tipCoords = fliplr(tipFinder(thisSamp, CLIP));
tipCoords = [(tipCoords(1) + ul(1)-1), (tipCoords(2) + ul(2)-1)];   

% Draw line from skeleton end to tip
[lineidx2] = drawline(tipEndCoord,tipCoords,size(CS)); 
CS(lineidx) = 1;
CS(lineidx2) = 1;





end

% Copyright (c) 2013,  Logan Scott Johnson
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
% 
%     Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
%     Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
%     Neither the name of the University of Wisconsin, Madison nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
