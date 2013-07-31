function [thisI, boundbox] = cropBlob(SZ, P, PCAwind ) 
% cropBlob takes in a regionprops PixelIndexList, size of the image, and a
%   dilating constant.  It returns a cropped patch of the image that leaves
%   extra room on the edges by dialating the binary with a large filter.
%   It also returns a regionprops BoundingBox used to crop the image.
%
% SZ : 1 by 2 vector indicating dimensions of the image
% P : regionprops PixelIndexList of the binary image in non structure
% PCAwind : Constant used to dilate the image
%
% thisI : Cropped image
% boundbox : regionprops BoundingBox used to crop the image

thisI = zeros(SZ);
thisI(P) = 1; 

% Dilate binary, find boundary, crop
SESZ = PCAwind * 2;
dI = imdilate(thisI, strel('disk',SESZ));
L =  bwlabel(dI);
dIP = regionprops(L,'BoundingBox');

boundbox = dIP(1).BoundingBox;
bb.ulxy = [single(ceil(boundbox(1))), single(ceil(boundbox(2)))];

bb.XwidthYwidth = [single(ceil(boundbox(3))), single(ceil(boundbox(4)))];
tmp = bb.XwidthYwidth + bb.ulxy;
tmp2 = fliplr(size( thisI));
tmp3 = (tmp > tmp2);
bb.ulxy = bb.ulxy - tmp3;
boundbox = bb;
cropI1 = thisI(  boundbox.ulxy(2) : (boundbox.ulxy(2)+boundbox.XwidthYwidth(2)) , boundbox.ulxy(1): (bb.ulxy(1)+boundbox.XwidthYwidth(1)) );
thisI = cropI1;

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
