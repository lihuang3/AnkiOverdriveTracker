workingDir = './RaceV05';

imageNames = dir(fullfile(workingDir,'*.png'));
imageNames = {imageNames.name}';

outputVideo = VideoWriter('shuttle_out.mp4');
outputVideo.FrameRate = 29;
open(outputVideo)

for ii = 1:length(imageNames)
   img = imread(fullfile(workingDir,imageNames{ii}));
   writeVideo(outputVideo,img)
end


close(outputVideo)
