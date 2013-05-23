function [tipEndIdx] = getTipEnd(tipCoords, SZ, ends)
% getTipEnd determines the closest matix index in ends to tipCoords
%
% tipCoords : 1 by 2 or 2 by 1 vector of xy coordinates
% SZ : 1 by 2 vector indicating dimensions of matrix to index from
% ends : matrix indexes to search
%
% tipEndIdx : index to matrix which is closest to tipCoords

endsCoords = [];
for i = 1:numel(ends)
    endsCoords = [endsCoords; getCoords(ends(i),SZ)];        
end
endsCoordsDists = [];
for i = 1:size(endsCoords,1)
    tisdis = sqrt((((abs(endsCoords(i,1)-tipCoords(1)))^2) + ((abs(endsCoords(i,2)-tipCoords(2)))^2)));
    endsCoordsDists = [endsCoordsDists; tisdis];
end
closestDist = min(endsCoordsDists,[],1);
closesCoordIdx = find(endsCoordsDists == closestDist);
closesCoordIdx = closesCoordIdx(1);
tipEndIdx = ends(closesCoordIdx);


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
