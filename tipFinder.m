function [TIP] = tipFinder(I,CLIP)
% tipFinder takes an binary image sample around a root tip and returns the
%   xy coordinates of the tip
%
% I : Binary image to search for tip in
% CLIP : Amount to clip off the boundary trace to more accurately find the
%           tip
%
% TIP : 1 by 2 vector indicating the XY coordinates of the tip


II = zeros(size(I));

% Find an edge pixel to do a boundary trace
II(:,1) = 1;
II(1,:) = 1;
II(:,end) = 1;
II(end,:) = 1;
III = and(I,II);
startidx = find(III);
startidx = startidx(1);
startcoord = getCoords(startidx,size(I));
B = bwtraceboundary(I,startcoord,'S');          

% Remove pixels on edge of image
B(find(B(:,1) == 1),:) = [];
B(find(B(:,2) == 1),:) = [];
B(find(B(:,1) == size(I,1)),:) = [];
B(find(B(:,2) == size(I,2)),:) = [];


% Get the curvature along the boundary
idx = find(B(:,1) == 1);
B(idx,:) = [];
C1 = cwt(B(:,1),[25],'gaus2');
C2 = cwt(B(:,2),[25],'gaus2');
D1 = cwt(B(:,1),[25],'gaus1');    
D2 = cwt(B(:,2),[25],'gaus1');
TAN = [D1' D2'];
for i = 1:size(TAN,1)
    TAN(i,:) = TAN(i,:) / norm(TAN(i,:));
end
NOR = [-TAN(:,2) TAN(:,1)];
ACC = [C1' C2'];
N = sum((NOR.*ACC),2);

% Find the peak curvature in the area of interest

[JUNK idx] = min(N(CLIP:end-(CLIP)));
TIP = [B(idx+(CLIP),1) B(idx+(CLIP),2)];
TIP = fliplr(TIP);


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


