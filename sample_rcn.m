function [S Q] = sample_rcn(r,c,I,W)
% sample_rcn samples a matix patch and returns it and the reshaped version

% r : row
% c : column
% I : image
% W : width
% 
% S : each row is an element of dim3
% Q : S but catenated along dim 3

SE = strel('square',W);                 % make square of size W
[d1 d2] = getneighbors(SE);             % get nhood
RAD = (W-1)/2;                          % half square value
I = padarray(I,[RAD RAD],0);            % pad array
S = [];                 
Q = [];
for p = 1:size(r,1)
    temp = zeros(W);                    %
    idx1 = d1(:,1) + r(p) + RAD;        
    idx2 = d1(:,2) + c(p) + RAD;
    IND = sub2ind(size(I),idx1,idx2);
    temp(1:size(IND)) = I(IND);

    S = [S;reshape(temp,[1 prod(size(temp))])];
    Q = cat(3,Q,temp);
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






