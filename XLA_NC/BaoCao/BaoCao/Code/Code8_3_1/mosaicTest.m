clear;
close all;
%config;
%f = 'a';
f = 'b';
ext = 'jpg';
% img1 = imread([f '1.' ext]);
% img2 = imread([f '1.' ext]);

img1 = imread('a1.jpg');
img2 = imread('a2.jpg');
% img3 = imread('b3.jpg');

img0 = imMosaic(img2,img1,1);
% img0 = imMosaic(img1,img2,1);
%img0 = imMosaic(img1,img0,1);
figure,imshow(img0)
imwrite(img0,'Result_8_3_1.jpg','jpg')