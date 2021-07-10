%xyloObj = VideoReader('test.avi'); %read video -> Can not find test.avi
%replace: rhinos.avi
xyloObj = VideoReader('rhinos.avi');
nFrames = xyloObj.NumberOfFrames; %number of frames
count = 1; %count the extracted frames
letter = 'a'; %tag，to make subsequent read sequences normal, precede the Arabic numerals with letters

for k = 1:5:nFrames
    mov(k).cdata = read(xyloObj, k); %image color data
    strtemp = strcat('images/', letter + count / 10);
    strtemp = strcat(strtemp, int2str(count));
    strtemp = strcat(strtemp, '.jpg');
    count = count + 1;
    imwrite(mov(k).cdata, strtemp); %save as strtemp.jpg
end

%feature transformation method based on color——using color histogram to measure color feature
%pop up a few key frames
filenames = dir('images/*.jpg'); %image source
num = size(filenames, 1); %number of images
key = zeros(1, num); % (0,1) key frame array
count = 0; %save a few key frames
threshold = 0.75; %set threshold

if num == 0
    error('Sorry, there is no pictures in images folder!');
else %set the first frame as key frame
    img = imread(strcat('images/', filenames(1).name));
    key(1) = 1;
    count = count + 1;
    %obtain RGB histogram
    [preCountR, x] = imhist(img(:, :, 1)); %red histogram
    [preCountG, x] = imhist(img(:, :, 2)); %green histogram
    [preCountB, x] = imhist(img(:, :, 3)); %blue histogram
    %show first key frame
    figure(count);
    imshow('images/a1.jpg');

    for k = 2:num
        img = imread(strcat('images/', filenames(k).name));
        [newCountR, x] = imhist(img(:, :, 1)); %red histogram
        [newCountG, x] = imhist(img(:, :, 2)); %green histogram
        [newCountB, x] = imhist(img(:, :, 3)); %blue histogram
        sR = 0;
        sG = 0;
        sB = 0;

        %use method of color histograms
        for j = 1:256
            sR = min(preCountR(j), newCountR(j)) + sR;
            sG = min(preCountG(j), newCountG(j)) + sG;
            sB = min(preCountB(j), newCountB(j)) + sB;
        end

        dR = sR / sum(newCountR);
        dG = sG / sum(newCountG);
        dB = sB / sum(newCountB);
        %YUV,persons are sensitive to Y
        d = 0.30 * dR + 0.59 * dG + 0.11 * dB;

        if d < threshold %small similarity, new keyframes found
            key(k) = 1; %set as keyframes
            count = count + 1;
            figure(count);
            imshow(strcat('images/', filenames(k).name));
            %nearest update color histogram
            preCountR = newCountR;
            preCountG = newCountG;
            preCountB = newCountB;
        end

    end

end

keyFrameIndexes = find(key)
