function [thisML thiscurv thiscurvgood failed] = prepOutput(saveMidline, saveDeriv, saveCurvature, res)
% prepOutput prepares data structure for writing outputs
%
% saveMidline : boolean...obvious
% saveDeriv : boolean...obvious
% saveCurvature : boolean...obvious
% res : Results data structure
%
% thisML : Prepared midline
% thiscurv : Prepared curvature
% thiscurvgood : Prepared first derivative
failed = 0;
thisML = {};
thiscurv = {};
thiscurvgood = {};


% Prep midlines for output
if(saveMidline)
   try
        for ii = 1:size(res,2)
            maxlen = 0;
            for i = 1:size(res{ii}.Mline,2)
                if(size(res{ii}.Mline{i},1) > maxlen)
                    maxlen = size(res{ii}.Mline{i},1);
                end
            end
            thisML{ii} = zeros( maxlen,(size(res{ii}.Mline,2)*2));
            for i = 1:2:(size(res{ii}.Mline,2)*2)
                i
                ceil(i/2)
                thisMLlen = size(res{ii}.Mline{ceil(i/2)},1);
                thisML{ii}( 1:thisMLlen ,i:(i+1)) = res{ii}.Mline{ceil(i/2)};
            end   
        end
   catch Exp
         fprintf(['EXP.identifier = ' Exp.identifier])
         fprintf(['EXP.message = ' Exp.message])
         csvwrite('RESULT',9);
         failed = 1; 
         return       
   end
    
    
end

% Prep curvature for output
if(saveDeriv)
    try
        for ii = 1:size(res,2)
            maxlen = 0;
            for i = 1:size(res{ii}.curv,2)
                if(size(res{ii}.curv{i},2) > maxlen)
                    maxlen = size(res{ii}.curv{i},2);
                end
            end
            thiscurv{ii} = zeros( maxlen,(size(res{ii}.curv,2)));
            for i = 1:size(res{ii}.curv,2)
                thiscurvlen = size(res{ii}.curv{i},2);
                thiscurv{ii}( 1:thiscurvlen ,i) = res{ii}.curv{i};
            end    
        end
    catch Exp
         fprintf(['EXP.identifier = ' Exp.identifier])
         fprintf(['EXP.message = ' Exp.message])
         csvwrite('RESULT',9);
         failed = 1;
         return
    end
end

if(saveCurvature)
    try
        for ii = 1:size(res,2)
            maxlen = 0;
            for i = 1:size(res{ii}.curvgood,2)
                if(size(res{ii}.curvgood{i},2) > maxlen)
                    maxlen = size(res{ii}.curvgood{i},2);
                end
            end
            thiscurvgood{ii} = zeros( maxlen,(size(res{ii}.curvgood,2)));
            for i = 1:size(res{ii}.curvgood,2)
                thiscurvlen = size(res{ii}.curvgood{i},2);
                thiscurvgood{ii}( 1:thiscurvlen ,i) = res{ii}.curvgood{i};
            end    
        end
    catch Exp
         fprintf(['EXP.identifier = ' Exp.identifier])
         fprintf(['EXP.message = ' Exp.message])
         csvwrite('RESULT',9);
         failed = 1;
         return
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
