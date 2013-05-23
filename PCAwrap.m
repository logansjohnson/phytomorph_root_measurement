function [ALLBVO, ALLBV, TangB ] = PCAwrap(PCAwind, tipCoords,BI, ALLBVO, ALLBV,jjj, iii )
% PCAwrap does pca window shrinking and angle fixing to measure the angle
%   of the root tip.
% 
% PCAwind : constant used to determine the size of the crop we use when
%   fitting PC's
% tipCoords : 1 by 2 vector of XY coordinates of the root tip
% BI : Binary image of root
% ALLBVO : Cellarray of original basis vectors
% ALLBV : Cellarray of adjusted basis vectors
% jjj : jjjth image
% iii : iiith root
%
% ALLBVO : Cellarray of original basis vectors
% ALLBV : Cellarray of adjusted basis vectors
% TangB : Angle of root tip



% sample tip and PCA binary image.
for i =  PCAwind:-10:(PCAwind-60)
    try
        
        
        NNN = i + 1;                                            % n-hood size of the tip patch                              
        HHH =(NNN-1)/2;                                         % width of the hood size
        [n1 n2] = ndgrid(-HHH:HHH,-HHH:HHH);
        n1 = reshape(n1,[1 numel(n1)]);
        n2 = reshape(n2,[1 numel(n2)]);
        NNNN = [n1;n2];
        [S Q] = sample_rcn(tipCoords(1),tipCoords(2),BI,NNN);
        V = [NNNN(1,:);NNNN(2,:)];
        radi = NNNN(1,:).^2 +  NNNN(2,:).^2 < (NNN/2)^2;
        fidxGOOD = find(S & radi);
        [SIM U BV LV C ERR] = PCA_FIT(V(:,fidxGOOD)',2);       % PCA fit the tip       
        ALLBVO{jjj}{end+1} = BV;        
        if(iii == 1) %If its the first image, we know how to flip the angle to make it look nice
            if(BV(1,2) < 0 )
                TangB = (atan2(-BV(1,1),-BV(1,2))*180/pi)*-1; 
                ALLBV{jjj}{end+1} = -BV;
            else
                TangB = (atan2(BV(1,1),BV(1,2))*180/pi)*-1; 
                ALLBV{jjj}{end+1} = BV;
            end
        else
             %Flip angle to make it look nice
             ttbv = ALLBV{jjj}{iii-1};              
             log1 = ( abs(ttbv(1,2) + BV(1,2))  <   abs(ttbv(1,2) + -BV(1,2)) );  
             log2 = (abs(ttbv(1,1) + BV(1,1))  <   abs(ttbv(1,1) + -BV(1,1)) );
             log3 = (BV(1,1) < 0) && (ttbv(1,1) >= 0); %true if dif sign 1st neg
             log4 = (BV(1,2) < 0) && (ttbv(1,2) >= 0);%true if dif sign 2nd neg
             log5 = (BV(1,1) >= 0) && (ttbv(1,1) < 0); %true if dif signs 1st pos
             log6 = (BV(1,2) >= 0) && (ttbv(1,2) < 0); %true if dif signs 2nd pos
             log7 = log3 || log5;
             log8 = log4 || log6;
             log9 = (log7 && log8); %true if both BV's changed sign...
             log10 = (log9 && (log1 || log2)) || (log1 && log2); 
             nlog1 = abs(ttbv(1,2) + -BV(1,2)) + abs(ttbv(1,1) + -BV(1,1)); %cost of not flipping
             nlog2 = abs(ttbv(1,2) + BV(1,2)) + abs(ttbv(1,1) + BV(1,1)) ; %cost of flipping
             log11 = nlog1 > nlog2;
             log10 = (log9 && (log1 || log2)) || (log11); 
             if( log10)
                 TangB = (atan2(-BV(1,1),-BV(1,2))*180/pi)*-1; 
                ALLBV{jjj}{end+1} = -BV; 
             else
                TangB = (atan2(BV(1,1),BV(1,2))*180/pi)*-1;
                ALLBV{jjj}{end+1} = BV;
             end              
        end
        %If we got here we did it!
        break
    catch Exp
        if(i==(PCAwind-60))
            % window is too small...give up
            csvwrite('RESULT',11)
            exit
        end
    end
end


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
