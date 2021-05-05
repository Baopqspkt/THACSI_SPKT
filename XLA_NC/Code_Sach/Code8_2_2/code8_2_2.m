%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
A=imread('source_1.bmp');
subplot(1, 2, 1), imshow(A)
title(' Source Image A')
B=imread('source_2.bmp');
subplot(1, 2, 2), imshow(B)
title('Source Image B')
%%%%%%%%%%%%%%%%%%%%%%%%%%
[high, wid] = size(A);
A1=double(A);
B1=double(B);
sub_A=A1(high/2-39:high/2,3*wid/4:3*wid/4+39);
% sub_A=A1(high/2-39:high/2,end-39:end);
sub_B1=B1(11:50, 11:50);
mod1=sub_A - sub_B1;
mat1=sum(sum(mod1 .* mod1));
mat_best = mat1;
for x1 = 1:40:wid - 40
    for y1 = 1:40:high - 40
        sub_B = B1(y1:y1 + 39, x1:x1 + 39);
        mod = sub_A - sub_B;
        mat = sum(sum(mod .* mod));
        if mat <= mat_best
            mat_best = mat;
            xx = x1;
            yy = y1;
        end
    end
end
x = xx; % custom settings
y = yy;
for x2 = xx - 40:xx
    for y2 = yy - 20:yy + 80
        sub_B2 = B1(y2:y2 + 39, x2:x2 + 39);
        mod2 = sub_A - sub_B2;
        mat2 = sum(sum(mod2 .* mod2));
        if mat2 <= mat_best
            mat_best = mat2;
            x = x2;
            y = y2;
        end
    end
end
% figure
% colormap(gray);
% subplot(2,1,1);imagesc(sub_A)
% subplot(2,1,2);imagesc(sub_B2)
%%%%%%%%%%%%%%%%%%%%%%%%%%
x = 55; y = 1;
%************************
if y == 0
    AA = A(1:high - 1, 1:3 * wid / 4);
    BB = B(1:high - 1, 1:wid - 1);
else
    if y >= high / 2
        AA = A(1:high - y, 1:3 * wid / 4);
        BB = B(y:high - 1, x:wid - 1);
    else
        AA = A(y:high - 1, 1:3 * wid / 4);
        BB = B(y:high - y, x:wid - 20);
    end
end

C = [AA BB];
%imwrite(C,'Directly stitched image 34.bmp');
figure, imshow(C)
title(' Directly stitched image')
if y == 0
    A2 = A(1:high - 1, :);
    B2 = B(1:high, :);
else
    if y>=50
        A2=A(1:high - y, :);
        B2=B(y:high - 1, :);
    else
        A2=A(y:high - 1, :);
        B2=B(1:high - y, :);
    end
end

x = 140; %************************
[high2, wid2] = size(A2);
a1 = A2(1:high2, wid - x + 1:wid);
b1 = B2(1:high2, 1:x);
a = double(a1);

b = double(b1);
d1O = linspace(1, 0, x);
d = 1:high2;
d1 = d1O';
[X1, y1] = meshgrid(d1, d);
im1 = a .* X1; %************************
d20 = linspace(0, 1, x);
d2 = d20';
[X2, y2] = meshgrid(d2, d);
im2 = b .* X2;
im11 = uint8(im1);
im22 = uint8(im2);
im3 = imadd(im11, im22);
figure, imshow(im3)
title('Fusion zone with gradated')
a_b = imadd(im11, im22);
aa = A2(1:high2, 1:wid - x);
bb = B2(1:high2, x:wid);
%D2=[aa a_b bb];
D2 = [aa(:, 1:wid2 - x) a_b bb]; %************************
imwrite(D2, '6.bmp');
figure, imshow(D2)
title('Image with gradated');