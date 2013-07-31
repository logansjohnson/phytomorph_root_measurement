function [CS] = cleanSkeleton(BI, SPUR)
% cleanSkeleton takes a binary image of a root and returns a skeleton that
%   should have no spurs and is connected
%
% BI : Binary image to compute skeleton on
% SPUR : Constant used for despurring
%
% CS : Binary of the clean skeleton

% Estimate midline using skeletonization
BI = padarray(BI,[1 1]);
BI = bwmorph(BI,'skel',inf);
%S = BI(2:end-1,2:end-1); % Original skeleton
% Remove spurs
BI = bwmorph(BI,'spur',SPUR);
BI = bwmorph(BI,'skel',inf);

CS = BI(2:end-1,2:end-1);
S = padarray(CS,[1, 1]);
CSorig = S;
S = padarray(S,[1, 1]);
SZ = size(CSorig);
N = im2col(S, [3 3], 'Sliding');
sumN = sum(N,1);
sumN = reshape((sumN),SZ(1),SZ(2));
ends = find( (and(sumN == 2, CSorig == 1)) );
inters = find( (and(sumN > 3, CSorig == 1)) );
CSorig(inters) = 0;
L = bwlabel(CSorig,8);
R = regionprops(L,'Area','PixelIdxList');
[big,bigIdx] = max([R.Area]);
CSorig = zeros(size(CSorig));
CSorig( R(bigIdx).PixelIdxList) = 1;

BI = padarray(CSorig,[1 1]);

BI = bwmorph(BI,'spur',1);
BI = bwmorph(BI,'skel',inf);
CS = BI(3:end-2,3:end-2); % Original clean skeleton



% Relabel CS as largest connected piece of skeleton
L = bwlabel(CS,8);
R = regionprops(L,'Area','PixelIdxList');
[big,bigIdx] = max([R.Area]);
CS = zeros(size(CS));
CS( R(bigIdx).PixelIdxList) = 1;


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
