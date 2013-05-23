function [P Pidx SZ] = isolateRoots(close,I,minPix4Root2, filterSize, filterCount)
% isolateRoots reads an image and segments it returning the regionprops
%   PixelIndexList, and the indices to it indicating it is reasonable.
%
% close : size of greyscale filtering disc
% I : file path to the image
% minPix4Root2 : minimum number of pixels to label a blob as good
% filterSize : size of binary filtering disc
% filterCount : number of times to filter greyscale
%
% P : regionprops PixelIndexList of segmented blobs
% Pidx : indices to P of good blobs
% SZ : 1 by 2 vector indicating size of image

% Read image, get info
I = imread(I);
I = I(2:(end-1),2:(end-1));
SZ = size(I);
I2 = I;

%filter filterCount times
for i = 1:filterCount
    BK = imclose(I2,strel('disk',close));
    I2 = double(I2) - double(BK);
    I2 = imfilter(I2,fspecial('disk',filterSize),'replicate');
    I2 = I2 - min(I2(:));
    I2 = I2 / max(I2(:));
    I2 = I2 * 255;
    I2 = round(I2);
    I2 = uint8(I2);
end

% Binarize
level = graythresh(uint8(I2));
I3 = ~im2bw(I2, level);

% Filter Binary
I3 = imdilate(I3,strel('disk',filterSize));
I3 = imerode(I3,strel('disk',filterSize));
I3 = imfill(I3,'holes');

% Find Blobs
L = bwlabel(I3);

% Eliminate Bad Blobs
P = regionprops(L,'Area', 'PixelIdxList','BoundingBox');
idx = find([P.Area] > minPix4Root2); % Too small
%Check edge intersection
Z = zeros(size(I));
Z(1,:) = 1;
Z(end,:) = 1;
Z(:,1) = 1;
Z(:,end) = 1;
Zidx = find(Z);
badidx = [];
for i = 1:numel(idx)
    isect = intersect(Zidx, P(idx(i)).PixelIdxList);
    if(numel(isect)<1)
       badidx = [badidx; i]; 
    end   
end
idx(badidx) = [];
Pidx = idx;



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
