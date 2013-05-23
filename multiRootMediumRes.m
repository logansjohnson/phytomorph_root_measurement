function [] = multiRootMediumRes(InPath,rtWidth,scale,saveAngle, saveLength, saveTip, saveDeriv, saveCurvature, saveMidline, saveImage)
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

cpstring = 'Copyright (c) 2013,  Logan Scott Johnson\n All rights reserved.\n \n Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:\n \n     Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.\n     Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.\n     Neither the name of the University of Wisconsin, Madison nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.\n \n THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n';
fprintf(cpstring);

% multiRootMediumRes takes multiple measurements on multiple roots in a time series of tifs
%   It should work on a unix type environment with MATLAB R2011b or maybe greater
%
% InPath : file path to image sequence (may need trailing slash)  unix
%           style pathing only.  It looks for numerically named 'tif',
%           'TIF', 'tiff' or 'TIFF's.  It then sorts them numerically.  If
%           the file name not including the extension is not numeric it
%           will not work.
% rtWidth : root width in pixels
% scale : scaling factor for a single dimension (Will cause recalulation of
%           constants)  The algorithm will scale the image by this factor
%           before doing any calculations on it.  It will also recalculate
%           rtWidth so make sure to consider that.
% saveAngle : boolean...obvious
% saveLength : boolean...obvious
% saveTip : boolean...obvious
% saveDeriv : boolean...obvious
% saveCurvature : boolean...obvious
% saveMidline : boolean...obvious
% saveImage : boolean...obvious


%For the sample data set the following two inputs should be used.
%rtWidth = 15;
%scale = 1;

rtWidth = round(rtWidth);

minPix4Root = (rtWidth^2)*(10/3);
minPix4Root2 = round(minPix4Root * scale *scale);

rtWidth = rtWidth * scale;

PCAwind = round((rtWidth * 4)/10)*10;
close = rtWidth * 4;
SPUR=round(rtWidth*(5/6));
DILATEERODE = round(rtWidth/2);
CLIP = round(rtWidth*(10/3));
WINDOWSIZE = round(rtWidth*(8/3));
filterSize = round(rtWidth/3);
filterCount = 3;

try 
    [cdir] = sortImages(InPath);
catch Exp
    csvwrite('RESULT',1)
    exit
end


ALLBVO = {};
ALLBV = {};
for i = 1:size(cdir,1)
    
    fprintf(num2str(i))

    try
        [P Pidx SZ] = isolateRoots(close,[InPath cdir(i).name],minPix4Root2, filterSize, filterCount);
    catch Exp
        csvwrite('RESULT',12)
        exit
    end
    
    % If we are on the first image, we can't match the roots to the old img
    if(i==1)
       oldP = P;
       oldPidx = Pidx;
       numRoots = numel(Pidx);
    end
    if(numel(Pidx) ~= numRoots)
        exit
        %Break, new root in ith image or one disapeered... 
    end
    
    [Pidx] = reorderBlobs(oldPidx, Pidx,oldP,P);
    
    %For each root
    for j = 1:numel(Pidx)
        % Setup Data structures
        if( (i==1))
            res{j}.TangB = [];
            res{j}.Rlen = [];
            res{j}.tipCoords = [];
            res{j}.Mline = {};   
            res{j}.curv = {};
            res{j}.curvgood = {};
            ALLBVO{j} = {};
            ALLBV{j} = {};
        end
        
        % setup binary of single root.    

        [thisI, boundbox] = cropBlob(SZ, P(Pidx(j)).PixelIdxList, PCAwind);

        try
            
            %Get measurements
            [TangB  Rlen Mline tipCoords CURV CURVGOOD  IOUT ALLBVO ALLBV] = measureRoot(thisI,boundbox,PCAwind,ALLBVO,ALLBV,i,j, DILATEERODE, SPUR,CLIP,WINDOWSIZE);
 
            % store measurements and write annotated image.
            res{j}.TangB = [res{j}.TangB; TangB];
            res{j}.Rlen = [res{j}.Rlen; Rlen];
            res{j}.tipCoords = [res{j}.tipCoords; tipCoords];
            res{j}.curv{i} = CURV;
            res{j}.curvgood{i} = CURVGOOD;
            res{j}.Mline{i} = Mline;
            
            if(saveImage)
                imwrite(IOUT, ['MLOUT_' num2str(j) '_' cdir(i).name])
            end
        catch Exp
            csvwrite('RESULT',8)
            fprintf(['EXP.identifier = ' Exp.identifier])
            fprintf(['EXP.message = ' Exp.message])
            exit       
        end
            
        
    end
    
    
end

[thisML thiscurv thiscurvgood] = prepOutput(saveMidline, saveDeriv, saveCurvature, res);

% Write measurement csvs
try
    for ii = 1:size(res,2)
         if(saveAngle)
            csvwrite(['./angle_' num2str(ii) '.csv'],res{ii}.TangB)
         end
         if(saveLength)
             csvwrite(['./lengthML_' num2str(ii) '.csv'],res{ii}.Rlen)
         end
         if(saveTip)
             csvwrite(['./tip_' num2str(ii) '.csv'],res{ii}.tipCoords)
         end
         if(saveDeriv)
             csvwrite(['./curvature_' num2str(ii) '.csv'],thiscurv{ii})
         end
         if(saveCurvature)
             csvwrite(['./curvatureGOOD_' num2str(ii) '.csv'],thiscurvgood{ii})
         end
         if(saveMidline)
             csvwrite(['./midline_' num2str(ii) '.csv'],thisML{ii})
         end
    end
catch Exp
     fprintf(['EXP.identifier = ' Exp.identifier])
     fprintf(['EXP.message = ' Exp.message])
     csvwrite('RESULT',5);
     exit
end
csvwrite('RESULT',0);






















