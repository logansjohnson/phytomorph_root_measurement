function [ML, CURV,angOnCurv, newCS] = smoothML(CS, BI, tipCoords)
% smooth ML Smooths the midline using parametric spline fitting with 
%   phantom knots and bound knots
%
% CS : clean skeleton
% BI : Binarized image of root
% tipCoords: 1 by 2 vector of XY coordinates of the root tip
%
% ML : N by 2 matrix of XY coordinates of the midline
% CURV : Curvature along midline
% angOnCurv : angle of midline along the midline
% newCS : new clean skeleton after smoothing the midline

%find tip end and do some prep
warning off all
SZSZ = size(CS);
keepCS = CS;
S = padarray(CS,[1, 1]);
SZ = size(CS);
N = im2col(S, [3 3], 'Sliding');
sumN = sum(N,1);
sumN = reshape((sumN),SZ(1),SZ(2));
ends = find( (and(sumN == 2, CS == 1)) );
[ends] = getTipEnd(tipCoords, size(CS), ends);
[discoord] = getCoords(ends, size(CS))  ;
[ind, label] = drawline(tipCoords,discoord,size(CS));
CS(ind) = 1;
L =  bwlabel(CS);

% find non tip end and prep some matrices
P = regionprops(L,'Area','PixelIdxList','BoundingBox');
BB = ceil(P(1).BoundingBox);
if(BB(1)<1)
    BB(1) = 1;
end
if(BB(2)<1)
    BB(2) = 1;
end
if((BB(1)+BB(3)) > size(CS,2))
   BB(3) = BB(3) -1; 
end
if((BB(2)+BB(4)) > size(CS,1))
   BB(4) = BB(4) -1; 
end
CS = keepCS;
z = CS(BB(2):(BB(2)+BB(4)),BB(1):(BB(1)+BB(3)));
tipCoords = [(tipCoords(1) - (BB(2)-1)), (tipCoords(2) - (BB(1)-1))];
S = padarray(z,[1, 1]);
CSorig = S;
S = padarray(S,[1, 1]);
SZ = size(CSorig);
N = im2col(S, [3 3], 'Sliding');
sumN = sum(N,1);
sumN = reshape((sumN),SZ(1),SZ(2));
ends = find( (and(sumN == 2, CSorig == 1)) );
allends = ends;
S = CSorig;
[ends] = getTipEnd(tipCoords, SZ, ends);
if(ends(1) == allends(1))
    otherend = allends(2); 
else
    otherend = allends(1);
end
otherend = getCoords(otherend,SZ);



%Convert to x=f(t) and y=f(t)
curr = getCoords(ends(1),SZ);
prev = -1;
t = [0];
x = [curr(2)];
y = [curr(1)];
cnt = 1;
while(1)
    tS = S((curr(1)-1):(curr(1)+1),(curr(2)-1):(curr(2)+1));
    tS(2,2) = 0;
    fidx = find(tS);
    fidx2 = [];
    for i = 1:numel(fidx)
        fidx2 = [fidx2; getCoords(fidx(i),[3 3])];        
    end
    fidx3  = [];
    for i = 1:size(fidx2,1)
        fidx3 = [fidx3; [(fidx2(i,1) + curr(1)-2), (fidx2(i,2) + curr(2)-2)    ]];        
    end
    gonext=0;
    curr;
    prev;
    for i = 1:size(fidx3,1)
       if(~all(fidx3(i,:) == prev))
          prev = curr;
          curr = fidx3(i,:);
          tmp = sum(abs(prev - curr),2);
          t = [t, sqrt(tmp)];
          x = [x, curr(2)];
          y = [y, curr(1)];
          gonext=1;
          break;
       end
    end
    if(~gonext)
        break
    end
    cnt = cnt + 1;
end


%Add the tip coordinates if they are not yet at the beggining 
if(   ~((x(1) == tipCoords(2)) && y(1) == tipCoords(1)   ))
    x = [tipCoords(2) x];
    y = [tipCoords(1) y];
    t = [t(1) (((x(1) - x(2))^2 + (y(1) - y(2))^2)^.5) t(2:end)];
end


%find range of t
tt = [];
ttsum = 0;
for i = 1:numel(t)
    tt = [tt (ttsum + t(i))];
    ttsum = ttsum + t(i);
end


% add phantom coords
[x y tt] = addPhantom(x,y,tt,1);

% interpolate and filter to a constant dif(t)
interper = (  tt(1)):1:(tt(end));
yinterp = interp1(tt,y,interper);
xinterp = interp1(tt,x,interper);
h = fspecial('average', [1,round(numel(xinterp)*.05)]);
yinterp2 = imfilter(yinterp,h,'symmetric');
xinterp2 = imfilter(xinterp,h,'symmetric');

% prep the input to spline fitting, add coefs and knots
skip = 1:round(numel(yinterp2)*.02 ) :numel(yinterp2);
coefs = [yinterp2(skip); xinterp2(skip)];
knots = addKnots(interper(skip),6,6);
coefs = addCoefs(coefs,3,3);

% calculate spline
sp = spmak(knots,coefs);
pp = fn2fm(sp,'pp');
toeval = pp.breaks(1):.1:pp.breaks(end);
v = ppval(pp,toeval);

%find tip and end t's
vy = v(1,:);
vx = v(2,:);
tvx1 = abs(vx - tipCoords(2));
tvy1 = abs(vy - tipCoords(1));
tvxy1 = tvx1 + tvy1;
[junk,minI] = min(tvxy1);
tipT = toeval(minI(1));

tvx2 = abs(vx - otherend(2));
tvy2 = abs(vy - otherend(1));
tvxy2 = tvx2 + tvy2;
[junk,minI] = min(tvxy2);
seedT = toeval(minI(end));
toeval = round(tipT):1:round(seedT);
ppd = fnder(pp,1);
ppdd = fnder(pp,2);
v = ppval(pp,toeval);
vd = ppval(ppd,toeval);
vdd = ppval(ppdd,toeval);
vy = v(1,:);
vx = v(2,:);
vdy = vd(1,:);
vdx = vd(2,:);
vddy = vdd(1,:);
vddx = vdd(2,:);



newCS = zeros(size(CSorig));
%HERE WE FILL IN MIDLINE IMAGE
for i = 1:numel(v(2,:))
    try
        if(round(v(1,i))>size(newCS,1))
            % DO NOTHING
        elseif(round(v(2,i))>size(newCS,2))
            % DO NOTHING           
        else
            newCS(round(v(1,i)),round(v(2,i))) = 1;
        end
    catch exp
        % DO NOTHING estimated point out of range.
    end
end



%FIX MIDLINE TO BE spur free, spur 1
BI2 = padarray(newCS,[1 1]);
BI2 = bwmorph(BI2,'skel',inf);
BI2 = bwmorph(BI2,'spur',1);
BI2 = bwmorph(BI2,'skel',inf);
newCS = BI2(2:end-1,2:end-1);


%extract smoothed ML COORDS
newCS = newCS(2:(end-1),2:(end-1)); %adjust for padding
newCS2 = zeros(SZSZ);
newCS2(BB(2):(BB(2)+BB(4)),BB(1):(BB(1)+BB(3))) = newCS;
newCS = newCS2;
newCS = and(newCS,BI);
newCS(:,1) = 0;
newCS(1,:) = 0; 
newCS(:,end) = 0; 
newCS(end,:) = 0;
ML = find(newCS); 
MLC = [];
for i = 1:numel(ML)
    MLC = [MLC; (getCoords(ML(i),size(newCS)))];
end
ML = MLC;

%Calculate curvature while we are at it.
CURV = [];
for i = 1:numel(toeval)
    ttmp = ( ( ( vdx((i)) * vddy((i)) ) - ( vdy((i)) * vddx((i)) ) )  / ( ( ( vdx((i))^2 ) + ( vdy((i))^2 ) ) ^ 1.5 ) );    
    CURV = [CURV,  ttmp];    
end

%calculate the angle on the curv at each t
angOnCurv = atan2(vdy,vdx);

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

