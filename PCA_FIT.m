function [SIM U BV L COEFFS ERR] = PCA_FIT(M,COM)
% PCA_FIT does PCA fitting to find the angle of the input
%
% M : Matrix
% COM : Number of vectors to return
%
% BV : Basis vectors
% others unused

% Principal Component Analysis used for measuring angle of blob
U = mean(M,1);
for i = 1:size(M,1)
    M(i,:) = M(i,:) - U;
end

COV = cov(M);
[BV L] = eigs(double(COV),COM);
COEFFS = M*BV;
% Create the simulated signal
SIM = (BV*COEFFS')';
for i = 1:size(SIM,1)
    SIM(i,:) = SIM(i,:) + U;
end
ERR = sum((SIM - M).^2,2).^.5;

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
