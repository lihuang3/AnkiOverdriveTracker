% This document extract frames from a video and save frames as images

vid = VideoReader('RaceV05.mp4');
nFrames = 10000;

% vidWidth = vid.Width;
% vidHeight = vid.Height;

f1 = 0;
for f=1:nFrames
%       if (mod(f,4)==0)
%           f1 = f1+1;
  
  thisframe=read(vid,f);
  %thisfile=sprintf('C:\Users\lhuang28\Documents\GitHub\MagneticController\lihuang\imageprocessing\frame_%04d.jpg',f);
  thisfile = sprintf('./RaceV05/vid%05d.png',f1);
  img = imresize(thisframe, 2./3.);
  imwrite(img,thisfile);
  f1 = f1+1;  
end