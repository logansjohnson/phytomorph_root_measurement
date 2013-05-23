function [TangB Rlen Mline tipCoords CURV CURVGOOD IOUT ALLBVO ALLBV] = measureRoot(I, boundbox, PCAwind, ALLBVO, ALLBV, iii, jjj, DILATEERODE, SPUR, CLIP, WINDOWSIZE)
% measureRoot performs a variety of measurements on a binarized root
%
% I : Binarized image of root
% boundbox : regionprops BoundingBox used to crop the image
% PCAwind : constant used to determine the size of the crop we use when
%   fitting PC's
% ALLBVO : Cellarray of original basis vectors
% ALLBV : Cellarray of adjusted basis vectors
% jjj : jjjth image
% iii : iiith root
% DILATEERODE : constant used to clean the binary image
% SPUR : constant used to despur and clean skeleton
% CLIP : constant used to find the tip
% WINDOWSIZE : constant for completeskeleton
%
% TangB : Angle of root tip
% Rlen : Length of approximate midline (trust little)
% Mline : N by 2 matrix of XY coordinates of the midline
% tipCoords: 1 by 2 vector of XY coordinates of the root tip
% CURV : Curvature along midline
% CURVGOOD: angle of midline along the midline
% IOUT : Image of midline overlayed on binary
% ALLBVO : Cellarray of original basis vectors
% ALLBV : Cellarray of adjusted basis vectors


% Find blob
L = bwlabel(I,8);
P = regionprops(L,'area','PixelIdxList');
[JUNK pshiftVALUES.idx] = max([P.Area]);
I = zeros(size(I));
I(P(pshiftVALUES.idx).PixelIdxList) = 1;
BI = I;


% Filter the binary
BI = imerode(BI,strel('disk',DILATEERODE,0));
BI = imdilate(BI,strel('disk',DILATEERODE,0));

%special spur removal
[CS] = cleanSkeleton(BI, SPUR);

%now we redraw from end of skel to the edge.
[CS, tipCoords] = completeSkelton(BI, CS, CLIP, WINDOWSIZE);


%now we have an approx midline.
%PREP for smoothing ML.
[CS] = removeLineSpurs(CS);

%SMOOTH MidLine
try 
     [Mline, CURVGOOD,CURV, newCS] = smoothML(CS,BI,tipCoords);
catch Exp
     fprintf(['EXP.identifier = ' Exp.identifier])
     fprintf(['EXP.message = ' Exp.message])
     csvwrite('RESULT',14);
     exit
end

%Draw new midline
[IOUT] = drawImageFromCoords(Mline,BI);

Rlen = size(Mline,1);

%adjust midline for bounding box
for i = 1:size(Mline,1)
    Mline(i,:) = [ (Mline(i,1) + boundbox.ulxy(1)), (Mline(i,2) + boundbox.ulxy(2))];    
end

% Prep for PCA
BI = fliplr(BI');
tipCoords = fliplr(tipCoords);
tipCoords(2) = size(BI,2) - (tipCoords(2) - 1);

[ALLBVO, ALLBV, TangB ] = PCAwrap(PCAwind, tipCoords,BI, ALLBVO, ALLBV,jjj, iii );

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
