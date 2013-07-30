function [] = multiMultiRootMediumRes(InPath,rtWidth,scale,saveAngle, saveLength, saveTip, saveDeriv, saveCurvature, saveMidline, saveImage)
cdir = dir(InPath);
cdir(~[cdir.isdir]) = [];
cdir(1:2) = [];
for i = 1:size(cdir,1)       
   try
       fprintf([InPath cdir(i).name '\n'])
       multiRootMediumRes([InPath filesep cdir(i).name filesep],rtWidth,scale,saveAngle, saveLength, saveTip, saveDeriv, saveCurvature, saveMidline, saveImage)
       
   catch Exp
      fprintf(['failed on ' cdir(i).name]) 
   end
    
end
end

