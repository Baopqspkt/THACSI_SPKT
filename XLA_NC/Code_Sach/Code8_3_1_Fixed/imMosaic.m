function [ imgout ] = imMosaic( img1,img2,adjColor )
%[ imgout ] = imMosaic( img1,img2,adjColor )
%	img1 and img2 can (both) be rgb or gray, double or uint8.
%	If you have more than 2 images to do mosaic, call this function several
%	times.
%	If you set adjColor to 1, imMosaic will try to try to adjust the
%	color(for rgb) or grayscale(for gray image) of img1 linearly, so the 2 
%	images can join more naturally.
%	Yan Ke @ THUEE, 20110123, xjed09@gmail.com

% use SIFT to find corresponding points
[matchLoc1, matchLoc2] = siftMatch(img1, img2);

% use RANSAC to find homography matrix
[H, corrPtIdx] = findHomography(matchLoc2',matchLoc1');
H  %#ok
tform = maketform('projective',H');
img21 = imtransform(img2,tform); % reproject img2
figure,imshow(img1)
figure,imshow(img21)

% adjust color or grayscale linearly, using corresponding infomation
[M1, N1, dim] = size(img1);
[M2, N2, ~] = size(img2);
if exist('adjColor','var') && adjColor == 1
	radius = 2;
	x1ctrl = matchLoc1(corrPtIdx,1);
	y1ctrl = matchLoc1(corrPtIdx,2);
	x2ctrl = matchLoc2(corrPtIdx,1);
	y2ctrl = matchLoc2(corrPtIdx,2);
	ctrlLen = length(corrPtIdx);
	s1 = zeros(1,ctrlLen);
	s2 = zeros(1,ctrlLen);
	for color = 1:dim
		for p = 1:ctrlLen
			left = round(max(1,x1ctrl(p)-radius));
			right = round(min(N1,left+radius+1));
			up = round(max(1,y1ctrl(p)-radius));
			down = round(min(M1,up+radius+1));
			s1(p) = sum(sum(img1(up:down,left:right,color))); % ȡ���ܵ�ɫ��
		end
		for p = 1:ctrlLen
			left = round(max(1,x2ctrl(p)-radius));
			right = round(min(N2,left+radius+1));
			up = round(max(1,y2ctrl(p)-radius));
			down = round(min(M2,up+radius+1));
			s2(p) = sum(sum(img2(up:down,left:right,color)));
		end
		sc = (radius*2+1)^2*ctrlLen;
		adjcoef = polyfit(s1/sc,s2/sc,1);
		img1(:,:,color) = img1(:,:,color)*adjcoef(1)+adjcoef(2);
	end
end

% do the mosaic
pt = zeros(3,4);
pt(:,1) = H*[1;1;1];
pt(:,2) = H*[N2;1;1];
pt(:,3) = H*[N2;M2;1];
pt(:,4) = H*[1;M2;1];
x2 = pt(1,:)./pt(3,:);
y2 = pt(2,:)./pt(3,:);

up = round(min(y2));
Yoffset = 0;
if up <= 0
	Yoffset = -up+1;
	up = 1;
end

left = round(min(x2));
Xoffset = 0;
if left<=0
	Xoffset = -left+1;
	left = 1;
end

[M3, N3 ,~] = size(img21);
imgout(up:up+M3-1,left:left+N3-1,:) = img21;
	% img1 is above img21
imgout(Yoffset+1:Yoffset+M1,Xoffset+1:Xoffset+N1,:) = img1;

end

function [descriptors, locs] = sift(img)

	% If you have the Image Processing Toolbox, you can uncomment the following
	%   lines to allow input of color images, which will be converted to grayscale.
	if size(img,3)
	   img = rgb2gray(img);
	end
	
	[rows, cols] = size(img); 
	
	% Convert into PGM imagefile, readable by "keypoints" executable
	f = fopen('tmp.pgm', 'w');
	if f == -1
		error('Could not create file tmp.pgm.');
	end
	fprintf(f, 'P5\n%d\n%d\n255\n', cols, rows);
	fwrite(f, img', 'uint8');
	fclose(f);
	
	% Call keypoints executable
	if isunix
		command = './sift ';
	else
		command = 'siftWin32 ';
	end
	command = [command ' <tmp.pgm >tmp.key'];
	system(sprintf(command));
	
	% Open tmp.key and check its header
	g = fopen('tmp.key', 'r');
	if g == -1
		error('Could not open file tmp.key.');
	end
	[header, count] = fscanf(g, '%d %d', [1 2]);
	if count ~= 2
		error('Invalid keypoint file beginning.');
	end
	num = header(1);
	len = header(2);
	if len ~= 128
		error('Keypoint descriptor length invalid (should be 128).');
	end
	
	% Creates the two output matrices (use known size for efficiency)
	locs = double(zeros(num, 4));
	descriptors = double(zeros(num, 128));
	
	% Parse tmp.key
	for i = 1:num
		[vector, count] = fscanf(g, '%f %f %f %f', [1 4]); %row col scale ori
		if count ~= 4
			error('Invalid keypoint file format');
		end
		locs(i, :) = vector(1, :);
		
		[descrip, count] = fscanf(g, '%d', [1 len]);
		if (count ~= 128)
			error('Invalid keypoint file value.');
		end
		% Normalize each input vector to unit length
		descrip = descrip / sqrt(sum(descrip.^2));
		descriptors(i, :) = descrip(1, :);
	end
	%delete tmp.key
	fclose(g);
end
	
function [matchLoc1 matchLoc2] = siftMatch(img1, img2)
	% load matchdata
	% load img1data
	% load img2data
	%{,
	% Find SIFT keypoints for each image
	[des1, loc1] = sift(img1);
	[des2, loc2] = sift(img2);
	% save img1data des1 loc1
	% save img2data des2 loc2
		
	% For efficiency in Matlab, it is cheaper to compute dot products between
	%  unit vectors rather than Euclidean distances.  Note that the ratio of 
	%  angles (acos of dot products of unit vectors) is a close approximation
	%  to the ratio of Euclidean distances for small angles.
	%
	% distRatio: Only keep matches in which the ratio of vector angles from the
	%   nearest to second nearest neighbor is less than distRatio.
	distRatio = 0.6;   
		
	% For each descriptor in the first image, select its match to second image.
	des2t = des2';                          % Precompute matrix transpose
	matchTable = zeros(1,size(des1,1));
	for i = 1 : size(des1,1)
		dotprods = des1(i,:) * des2t;        % Computes vector of dot products
		[vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results
		
		% Check if nearest neighbor has angle less than distRatio times 2nd.
		if (vals(1) < distRatio * vals(2))
			matchTable(i) = indx(1);
		else
			matchTable(i) = 0;
		end
	end
	% save matchdata matchTable
	%}
		
	% Create a new image showing the two images side by side.
	img3 = appendimages(img1,img2);
		
	% Show a figure with lines joining the accepted matches.
	figure('Position', [100 100 size(img3,2) size(img3,1)]);
	colormap('gray');
	imagesc(img3);
	hold on;
	cols1 = size(img1,2);
	for i = 1: size(des1,1)
		if (matchTable(i) > 0)
			line([loc1(i,2) loc2(matchTable(i),2)+cols1], ...
			[loc1(i,1) loc2(matchTable(i),1)], 'Color', 'c');
		end
	end
	hold off;
	num = sum(matchTable > 0);
	fprintf('Found %d matches.\n', num);
		
	idx1 = find(matchTable);
	idx2 = matchTable(idx1);
	x1 = loc1(idx1,2);
	x2 = loc2(idx2,2);
	y1 = loc1(idx1,1);
	y2 = loc2(idx2,1);
		
	matchLoc1 = [x1,y1];
	matchLoc2 = [x2,y2];	
end	

function [f inlierIdx] = ransac1( x,y,ransacCoef,funcFindF,funcDist )
	%[f inlierIdx] = ransac1( x,y,ransacCoef,funcFindF,funcDist )
	%	Use RANdom SAmple Consensus to find a fit from X to Y.
	%	X is M*n matrix including n points with dim M, Y is N*n;
	%	The fit, f, and the indices of inliers, are returned.
	%
	%	RANSACCOEF is a struct with following fields:
	%	minPtNum,iterNum,thDist,thInlrRatio
	%	MINPTNUM is the minimum number of points with whom can we 
	%	find a fit. For line fitting, it's 2. For homography, it's 4.
	%	ITERNUM is the number of iteration, THDIST is the inlier 
	%	distance threshold and ROUND(THINLRRATIO*n) is the inlier number threshold.
	%
	%	FUNCFINDF is a func handle, f1 = funcFindF(x1,y1)
	%	x1 is M*n1 and y1 is N*n1, n1 >= ransacCoef.minPtNum
	%	f1 can be of any type.
	%	FUNCDIST is a func handle, d = funcDist(f,x1,y1)
	%	It uses f returned by FUNCFINDF, and return the distance
	%	between f and the points, d is 1*n1.
	%	For line fitting, it should calculate the dist between the line and the
	%	points [x1;y1]; for homography, it should project x1 to y2 then
	%	calculate the dist between y1 and y2.
	
	
	minPtNum = ransacCoef.minPtNum;
	iterNum = ransacCoef.iterNum;
	thInlrRatio = ransacCoef.thInlrRatio;
	thDist = ransacCoef.thDist;
	ptNum = size(x,2);
	thInlr = round(thInlrRatio*ptNum);
	
	inlrNum = zeros(1,iterNum);
	fLib = cell(1,iterNum);
	
	for p = 1:iterNum
		% 1. fit using  random points
		sampleIdx = randIndex(ptNum,minPtNum);
		f1 = funcFindF(x(:,sampleIdx),y(:,sampleIdx));
		
		% 2. count the inliers, if more than thInlr, refit; else iterate
		dist = funcDist(f1,x,y);
		inlier1 = find(dist < thDist);
		inlrNum(p) = length(inlier1);
		if length(inlier1) < thInlr, continue; end
		fLib{p} = funcFindF(x(:,inlier1),y(:,inlier1));
	end
	
	% 3. choose the coef with the most inliers
	[~,idx] = max(inlrNum);
	f = fLib{idx};
	dist = funcDist(f,x,y);
	inlierIdx = find(dist < thDist);
		
end
