function code8_2_3()
    clc; 
    clear;
    %bpham debug
    global image1
    global image2
    global image11
    global image22
    
    image1 = (imread('1.jpg')); 
    [h, w, r] = size(image1);
    image2 = (imread('2.jpg')); 
    T0(:, :, :, 1) = image1; T0(:, :, :, 2) = image2;
    subplot(121), imshow(image1); subplot(122), imshow(image2);
    
    image11 = multi_resolution(image1, 2);
    image22 = multi_resolution(image2, 2);

    [r1, c1, d1] = size(image11);
    [r2, c2, d2] = size(image22);
    % calculate phase correlation offsets
    tic
    fprintf(' calculate phase correlation offsets...');
    [i, j] = poc_2pow(image11, image22);
    coor_shift(1, 1) = i;
    coor_shift(1, 2) = j;
    coor_shift(2, 1) = 0; coor_shift(2, 2) = 0;
    coor_shift = coor_shift * 2^2; % % % convert to offsets of original image
    toc
    %transform into cylindrical coordinate system
    tic
    f = sqrt(h^2 + w^2);
    [T1, coor_shift02] = coortransf(T0, f, coor_shift);
    
    % bPHAM DEBUG
    % aa = T0(:, :, :, 1);
    % bb = T0(:, :, :, 2);
    % aaa = T1(:, :, :, 1);
    % bbb = T1(:, :, :, 2);
    toc
    %fuse overlapping areas
    tic
    fprintf('Fusing overlapping areas and stitching the image...');
    panorama1 = mosaic(T1(:, :, :, 1), T1(:, :, :, 2), coor_shift02(1, 1), coor_shift02(1, 2));
    toc

    %image reconstruction
    tic
    fprintf('Saving and displaying the result...');
    imwrite(panorama1, 'Result_8_3_3.jpg', 'jpg');
    imshow(panorama1, []);
    toc

function T = multi_resolution(Xb, n)
    % multiresolution decomposition
    [r1, c1, d1, N] = size(Xb);

    for i = 1:N
        Xb(:, :, 1, N) = filter2(fspecial('gaussian'), Xb(:, :, 1, N)); % %Default parameters of Gaussian filter [3 3]ˈ
        sigma = 0.5
        Xb(:, :, 2, N) = filter2(fspecial('gaussian'), Xb(:, :, 2, N));
        Xb(:, :, 3, N) = filter2(fspecial('gaussian'), Xb(:, :, 3, N));
    end

    step = 2^n;

    for i = 1:step:r1
        for j = 1:step:c1
            T((i + step - 1) / step, (j + step - 1) / step, :, :) = Xb(i, j, :, :);
        end
    end

function [dis, dm] = poc_2pow(imageL, imageR);
    %% phase correlation algorithm
    % imageL=image1;
    % imageR=image2;
    [H1, W1, d1] = size(imageL);
    [H2, W2, d2] = size(imageR);
    if d1 == 3 imageL = rgb2gray(imageL); 
    end %%%grayscale
    if d2 == 3 imageR = rgb2gray(imageR); 
    end
    % extract the binary outlines of the 2-exponential histogram
    [imageL, t1] = edge(imageL, 'canny', [], 1.2); % % % %sigma=1.2(Default 1)
    [imageR, t2] = edge(imageR, 'canny', [], 1.2); % % % % auto select the threshold value
    Xb = imageL; Yb = imageR;
        
    %2-exponential histogram
    for i = 5:11
        index2 = 2^i;
        if index2 <= H1 && index2 <= W1 h1 = index2; 
        end
        if index2 <= H2 && index2 <= W2 h2 = index2; 
        end
    end

    % minhw1=min(h1,w1);
    % minhw2=min(h2,w2);
    minhw1 = h1; minhw2 = h2;
    offset1 = round((H1 - minhw1) / 2);
    offset2 = round((H2 - minhw2) / 2);
    imageL = imageL(offset1:offset1 + minhw1 - 1, W1 - minhw1 + 1:W1); % % %choose right center in the left image
    imageR = imageR(offset2:offset2 + minhw2 - 1, 1:minhw2); % % % % choose left center in the right image

    % phase correlation algorithm for measuring offsets
    A = fft2(im2double(imageL)); %FFT in frequency domain
    B = fft2(im2double(imageR));
    AB = conj(A) .* (B); % % % conjugated convolution, equals to phase transformation
    modAB = abs(AB);
    %peak valueˈ˄IˈJ˅save peak coordinates which are offsets
    COR = ifft2(AB); % % %unnormalize, reverse transformation for coeerlation
    emin = 100000;
    % for i=1:10
    [maxC, sorti] = max(COR);
    [C, J] = max(maxC);
    I = sorti(J);

    if I < 20 dis = I;
    elseif H2 - I < 20 dis = (I - H2);
    else dis = 0;
    end
    dm = J;

function [T1, coor_shift02] = coortransf(T0, f, coor_shift)
    %%transformation from image coordinate to cylindrical coordinate
    %transform input image sequence T0 with focal length f
    coor_shift02 = coor_shift; % % % % the first dimension (row values) stay unchanged and the second dimension (column values) update after mapping
    [H, W, r, N] = size(T0);
    w2f = W / 2 / f;
    h2 = H / 2;
    constant2 = f * atan(W / (2 * f));
    constant1 = h2;

    for y = 1:W % % % % % % columns
        angle = atan(y / f - w2f); % % % %atan((y-W/2)/f);
        y1 = uint16(f * angle + constant2);
        if y1 == 0 y1 = 1; end
        for x = 1:H % % % % % % % % % % % % rows
            x1 = uint16((x - h2) * cos(angle) + constant1);
            if x1 == 0 x1 = 1; 
            end
            if r == 3 % % % % % % % % % % % % %color image
                for n = 1:N % % % % % % % % %
                    if (y == coor_shift(n, 2)) coor_shift02(n, 2) = y1; 
                    end %%%corresponding offsets
                    T1(x1, y1, :, n) = T0(x, y, :, n); % % % % mapping of points
                end
            elseif r == 1
            end
        end

    end

    [h, w, a, N] = size(T1);
    for i = 1:60
        for j = 1:w
            if (T1(i, j, :, :) == 0)
                T1(i, j, :, :) = 255;
            end

            if (T1(h - i, j, :, :) == 0)
                T1(i, j, :, :) = 255;
            end
        end

    end

function D = mosaic(image1, image2, i, j)
    [ra, ca, a] = size(image1);
    [rb, cb, b] = size(image2);
    Xa = image1; Ya = image2;
    % dis=i;%%% top and bottom offsets
    dis = i;
    EXa = zeros(abs(dis), ca, 3) + 255;
    EXb = zeros(abs(dis), cb, 3) + 255;

    if dis > 1
        Xa = [EXa; Xa];
        Ya = [Ya; EXb];
    elseif dis <- 1
        Xa = [Xa; EXa];
        Ya = [EXb; Ya];

    end

    dm = j; % % % stitching crack width ,limited no more than 50 pel
    A = Xa(:, 1:(ca - dm - 1), :);
    B1 = Xa(:, (ca - dm):ca, :);
    B2 = Ya(:, 1:dm, :);
    B = imagefusion02(B1, B2); % %partial overlapping(fusion)
    C = Ya(:, (dm + 1):cb, :); % % cut out the rest part of the second image

    D = [A, B, C]; % %merge and complete stitching
    %%%% eliminate accumulative errors
    [r, c] = size(D);

    if dis > 1
        D = D(1:(r - dis), :, :);
    elseif dis <- 1
        D = D((abs(dis) + 1):r, :, :);
    end

function [dis, dm] = phase_correlation(image1, image2);
    % phase correlation algorithm
    %%% C=phase_correlation(imagea,imageb) input two images
    [H1, W1, d1] = size(image1);
    [H2, W2, d2] = size(image2);
    if d1 == 3 image1 = rgb2gray(image1); 
    end %%%grayscale
    if d2 == 3 image2 = rgb2gray(image2); 
    end
    Xb = image1; Yb = image2;
    [image1, t1] = edge(image1, 'canny');
    [image2, t2] = edge(image2, 'canny');
    A = fft2(im2double(image1)); %FFT in frequency domain
    B = fft2(im2double(image2));
    AB = conj(A) .* (B); % % % conjugated convolution, equals to phase transformation
    COR = ifft2(AB); % % % unnormalize, reverse transformation for coeerlation
    [C, i] = max(COR); [C, J] = max(C); I = i(J); % % % peak valueˈ˄IˈJ˅save peak coordinates which are offsets

    if I < 15 dis = I;
    elseif H2 - I < 15 dis = I - H2;
    else dis = 0;
    end

    dm = J;

function C = imagefusion02(A, B)
    %%%image fusion
    [M, N, D] = size(A);

    if D == 3
        for i = 1:(N - 1)
            C(:, i, :) = round((double(A(:, i, :)) * (N - i) + double(B(:, i, :)) * i) / N);
        end

    elseif D == 1
        for i = 1:(N - 1)
            C(:, i) = round((double(A(:, i)) * (N - i) + double(B(:, i)) * i) / N);
        end
    end
    % figure,imshow(C/max(max(max(C))))
