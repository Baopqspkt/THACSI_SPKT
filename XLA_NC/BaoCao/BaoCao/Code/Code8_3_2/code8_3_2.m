%clc;
clear all;
% read image
pic1 = imread('a1.jpg');
pic2 = imread('a2.jpg');
% Harris feature points detection
points1 = myHarris(pic1);
points2 = myHarris(pic2);
% draw Harris feature points
figure(1)
drawHarrisCorner(pic1, points1, pic2, points2);
% describe Harris feature
des1 = myHarrisCornerDescription(pic1, points1);
des2 = myHarrisCornerDescription(pic2, points2);
% coarse match
matchs = myMatch(des1, des2);
% obtain position of match points
matchedPoints1 = points1(matchs(:, 1), :);
matchedPoints2 = points2(matchs(:, 2), :);
% line coarse match points
figure(2)
drawLinedCorner(pic1, matchedPoints1, pic2, matchedPoints2);
% Harris feature points fine matching
[newLoc1, newLoc2] = pointsSelect(matchedPoints1, matchedPoints2);
% line fine match points
figure(3)
drawLinedCorner(pic1, newLoc1, pic2, newLoc2);
% stitch images

im = picMatched(pic1, newLoc1, pic2, newLoc2);
% show the stitching image
figure(4)
imshow(im);
set(gcf, 'Color', 'w');
imwrite(im,'Result_8.3.2.jpg','jpg');


function points = myHarris(pic)
    % function:Harris feature points detection
    % input: RGB image or gray scale image
    % output:the row and col N×2 matrix of the feature point
    if length(size(pic)) == 3
        pic = rgb2gray(pic);
    end

    pic = double(pic);
    hx = [-1 0 1];
    Ix = filter2(hx, pic);
    hy = [-1; 0; 1];
    Iy = filter2(hy, pic);
    Ix2 = Ix .* Ix;
    Iy2 = Iy .* Iy;
    Ixy = Ix .* Iy;
    h = fspecial('gaussian', [7 7], 2);
    Ix2 = filter2(h, Ix2);
    Iy2 = filter2(h, Iy2);
    Ixy = filter2(h, Ixy);
    [heigth, width] = size(pic);
    alpha = 0.06;
    R = zeros(heigth, width);

    for i = 1:heigth

        for j = 1:width
            M = [Ix2(i, j) Ixy(i, j); Ixy(i, j) Iy2(i, j)];
            R(i, j) = det(M) - alpha * (trace(M)^2);
        end

    end

    Rmax = max(max(R));
    pMap = zeros(heigth, width);

    for i = 2:heigth - 1

        for j = 2:width - 1

            if R(i, j) > 0.01 * Rmax
                tm = R(i - 1:i + 1, j - 1:j + 1);
                tm(2, 2) = 0;

                if R(i, j) > tm
                    pMap(i, j) = 1;
                end

            end

        end

    end

    [row, col] = find(pMap == 1);
    points = [row, col];
end

function drawHarrisCorner(pic1, points1, pic2, points2)
    % function:draw Harris feature points’ match connection
    % input:
    % pic1�? pic2:the images need stitching
    % points1�? points2: Harris feature points position
    X1 = points1(:, 2);
    Y1 = points1(:, 1);
    X2 = points2(:, 2);
    Y2 = points2(:, 1);
    dif = size(pic1, 2);
    imshowpair(pic1, pic2, 'montage');
    hold on
    plot(X1, Y1, 'b*');
    plot(X2 + dif, Y2, 'b*');
    set(gcf, 'Color', 'w');
end

function des = myHarrisCornerDescription(pic, points)
    % Function: describe Harris feature points
    % Input:
    % pic:source image
    % points:feature points’ position
    % Output:
    % des: 8×N matrix describing Harris feature points
    if length(size(pic)) == 3
        pic = rgb2gray(pic);
    end

    len = length(points);
    des = zeros(8, len);

    for k = 1:len
        p = points(k, :);
        pc = pic(p(1), p(2));
        des(1, k) = pic(p(1) - 1, p(2) - 1) - pc;
        des(2, k) = pic(p(1), p(2) - 1) - pc;

        des(3, k) = pic(p(1) + 1, p(2) - 1) - pc;
        des(4, k) = pic(p(1) + 1, p(2)) - pc;
        des(5, k) = pic(p(1) + 1, p(2) + 1) - pc;
        des(6, k) = pic(p(1), p(2) + 1) - pc;
        des(7, k) = pic(p(1) - 1, p(2) + 1) - pc;
        des(8, k) = pic(p(1) - 1, p(2)) - pc;
        des(:, k) = des(:, k) / sum(des(:, k));
    end

end

function matchs = myMatch(des1, des2)
    % Function:feature point bidirectional match
    % input:
    % des1�? des2:feature point description matrix
    % Output:
    % matchs:correspondence relation of match points
    len1 = length(des1);
    len2 = length(des2);
    match1 = zeros(len1, 2);
    cor1 = zeros(1, len2);

    for i = 1:len1
        d1 = des1(:, i);

        for j = 1:len2
            d2 = des2(:, j);
            cor1(j) = (d1' * d2) / sqrt((d1' * d1) * (d2' * d2));
        end

        [~, indx] = max(cor1);
        match1(i, :) = [i, indx];
    end

    match2 = zeros(len2, 2);
    cor2 = zeros(1, len1);

    for i = 1:len2
        d2 = des2(:, i);

        for j = 1:len1
            d1 = des1(:, j);
            cor2(j) = (d1' * d2) / sqrt((d1' * d1) * (d2' * d2));
        end

        [~, indx] = max(cor2);
        match2(i, :) = [indx, i];
    end

    matchs = [];

    for i = 1:length(match1)

        for j = 1:length(match2)

            if match1(i, :) == match2(j, :)
                matchs = [matchs; match1(i, :)];
            end

        end

    end

end

function drawLinedCorner(pic1, loc1, pic2, loc2)
    % Function:draw connections between match points
    % Input:
    % pic1�? pic2:image to need stitching
    % loc1�? loc2:position of the paired points
    X1 = loc1(:, 2);
    Y1 = loc1(:, 1);
    X2 = loc2(:, 2);
    Y2 = loc2(:, 1);
    dif = size(pic1, 2);
    imshowpair(pic1, pic2, 'montage');
    hold on

    for k = 1:length(X1)
        plot(X1(k), Y1(k), 'b*');
        plot(X2(k) + dif, Y2(k), 'b*');
        line([X1(k), X2(k) + dif], [Y1(k), Y2(k)], 'Color', 'r');
    end

    set(gcf, 'Color', 'w');
end

function [newLoc1, newLoc2] = pointsSelect(loc1, loc2)
    % Filter:filter the paired match points and obtain the fine match points
    % Input:
    % loc1�? loc2:position of coarse match points
    % Output:
    % newLoc1�?newLoc2:position of fine match points
    slope = (loc2(:, 1) - loc1(:, 1)) ./ (loc2(:, 2) - loc1(:, 2));

    for k = 1:3
        slope = slope - mean(slope);
        len = length(slope);
        t = sort(abs(slope));
        thresh = t(round(0.5 * len));
        ind = abs(slope) <= thresh;
        slope = slope(ind);
        loc1 = loc1(ind, :);

        loc2 = loc2(ind, :);
    end

    newLoc1 = loc1;
    newLoc2 = loc2;
end

function im = picMatched(pic1, newLoc1, pic2, newLoc2)
    % Function: obtain the stitching image
    % Input:
    % pic1�? pic2: images need stitching
    % newLoc1�?newLoc2:new position of feature points
    % Output:
    % im: the stitching image
    if length(size(pic1)) == 2
        pic1 = cat(3, pic1, pic1, pic1);
    end

    if length(size(pic2)) == 2
        pic2 = cat(3, pic2, pic2, pic2);
    end

    SZ = 2000;
    X1 = newLoc1(:, 2);
    Y1 = newLoc1(:, 1);
    X2 = newLoc2(:, 2);
    Y2 = newLoc2(:, 1);
    sel = randperm(length(newLoc1), 3);
    x = X2(sel)';
    y = Y2(sel)';
    X = X1(sel)';
    Y = Y1(sel)';
    U = [x; y; ones(1, 3)];
    V = [X; Y; ones(1, 3)];
    T = V / U;
    cntrX = SZ / 2;
    cntrY = SZ / 2;
    im = zeros(SZ, SZ, 3);

    for i = 1:size(pic2, 1)

        for j = 1:size(pic2, 2)
            tmp = T * [j; i; 1];
            nx = round(tmp(1)) + cntrX;
            ny = round(tmp(2)) + cntrY;

            if nx >= 1 && nx <= SZ && ny >= 1 && ny <= SZ
                im(ny, nx, :) = pic2(i, j, :);

            end

        end

    end

    im = imresize(im, 1, 'bicubic');
    tpic1 = zeros(SZ, SZ, 3);
    tpic1(1 + cntrY:size(pic1, 1) + cntrY, 1 + cntrX:size(pic1, 2) + cntrX, :) = pic1;
    re = rgb2gray(uint8(im)) - rgb2gray(uint8(tpic1));

    for k = 1:3
        ta = im(:, :, k);
        tb = tpic1(:, :, k);
        ta(re == 0) = tb(re == 0);
        im(:, :, k) = ta;
    end

    clear ta tb re tpic1
    im = getPicture(im, SZ);
    im = uint8(im);

    if length(size(pic1)) == 2
        im = rgb2gray(im);
    end

end

function im = getPicture(pic, SZ)
    % Function: obtain the useful image region
    % Input
    % pic: the stitching image
    % SZ: given size
    % Output:
    % im: useful image region
    if length(size(pic)) == 2
        pic = cat(3, pic, pic, pic);
    end

    k = 1;

    while k < SZ

        if any(any(pic(k, :, :)))
            break
        end

        k = k + 1;
    end

    ceil = k; % Upper boundary
    k = SZ;

    while k > 0

        if any(any(pic(k, :, :)))
            break
        end

        k = k - 1;
    end

    bottom = k; % Lower boundary
    k = 1;

    while k < SZ

        if any(any(pic(:, k, :)))
            break
        end

        k = k + 1;
    end

    left = k; %left boundary
    k = SZ;

    while k > 0

        if any(any(pic(:, k, :)))
            break
        end

        k = k - 1;
    end

    right = k; %right boundary
    %%obtain image
    im = pic(ceil:bottom, left:right, :);
end
