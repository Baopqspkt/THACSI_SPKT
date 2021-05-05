
function points = code8_3_4(im)
%     clear all;
%     clc;
%     im=imread('lena256.pgm');
    % Extract keypoints using Harris algorithm (with an improvement
    % version)
    % Author :: Vincent Garcia
    % Date :: 05/12/2007
    % INPUT
    % im : the graylevel image
    % OUTPUT
    % points : the interest points extracted
    % REFERENCES
    % C.G. Harris and M.J. Stephens. "A combined corner and edge detector",
    % Proceedings Fourth Alvey Vision Conference, Manchester.
    % pp 147-151, 1988.
    % Alison Noble, "Descriptions of Image Surfaces", PhD thesis, Department
    % of Engineering Science, Oxford University 1989, p45.
    % C. Schmid, R. Mohrand and C. Bauckhage, "Evaluation of Interest Point Detectors",
    % Int. Journal of Computer Vision, 37(2), 151-172, 2000.
    % EXAMPLE
    % points = kp_harris(im)
    % only luminance value
    % im=imread('lena256.pgm');
    im=im2double(im);
    % im = double(im(:, :, 1));
    sigma = 1.5;
    % derivative masks
    s_D = 0.7 * sigma;
    x = -round(3 * s_D):round(3 * s_D);
    dx = x .* exp(-x .* x / (2 * s_D * s_D)) ./ (s_D * s_D * s_D * sqrt(2 * pi));
    dy = dx';
    % image derivatives
    Ix = conv2(im, dx, 'same');
    Iy = conv2(im, dy, 'same');
    % sum of the Auto-correlation matrix
    s_I = sigma;
    g = fspecial('gaussian', max(1, fix(6 * s_I + 1)), s_I);
    Ix2 = conv2(Ix.^2, g, 'same'); % Smoothed squared image derivatives
    Iy2 = conv2(Iy.^2, g, 'same');
    Ixy = conv2(Ix .* Iy, g, 'same');
    % interest point response
    cim = (Ix2 .* Iy2 - Ixy.^2) ./ (Ix2 + Iy2 + eps); % Alison Noble measure.
    % k = 0.06; cim = (Ix2.*Iy2 - Ixy.^2) - k*(Ix2 + Iy2).^2; % Original Harris measure.
    % find local maxima on 3x3 neighborgood
    [r, c, max_local] = findLocalMaximum(cim, 3 * s_I);
    % set threshold 1% of the maximum value
    t = 0.1 * max(max_local(:));
    % find local maxima greater than threshold
    [r, c] = find(max_local >= t);
    % build interest points
    points = [r, c];
    
    %bpham add 
    figure, imshow(im),title('Harris Feature Points');
    hold on
    plot(points(:,1),points(:,2),'r*');
end

function [row, col, max_local] = findLocalMaximum(val, radius)
    % Determine the local maximum of a given value
    %
    % Author :: Vincent Garcia
    % Date :: 09/02/2007
    %
    % INPUT
    % val : the NxM matrix containing values
    % radius : the radius of the neighborhood
    % OUTPUT
    % row : the row position of the local maxima
    % col : the column position of the local maxima
    % max_local : the NxM matrix containing values of val on unique local maximum
    % EXAMPLE
    % [l,c,m] = findLocalMaximum(img,radius);
    % FIND LOCAL MAXIMA BY DILATION (FAST) /!\ NON UNIQUE /!\
    % mask = fspecial('disk',radius)>0;
    % val2 = imdilate(val,mask);
    % index = val==val2;
    % [row,col] = find(index==1);
    % max_local = zeros(size(val));
    % max_local(index) = val(index);
    % FIND UNIQUE LOCAL MAXIMA USING FILTERING (FAST)
    mask = fspecial('disk', radius) > 0;
    nb = sum(mask(:));
    highest = ordfilt2(val, nb, mask);
    second_highest = ordfilt2(val, nb - 1, mask);
    index = highest == val & highest ~= second_highest;
    max_local = zeros(size(val));
    max_local(index) = val(index);
    [row, col] = find(index == 1);
    % FIND UNIQUE LOCAL MAXIMA (FAST)
    % val_height = size(val,1);
    % val_width = size(val,2);
    % max_local = zeros(val_height,val_width);
    % val_enlarge = zeros(val_height+2*radius,val_width+2*radius);
    % val_mask = zeros(val_height+2*radius,val_width+2*radius);
    % val_enlarge( (1:val_height)+radius , (1:val_width)+radius ) = val;
    % val_mask( (1:val_height)+radius , (1:val_width)+radius ) = 1;
    % mask = fspecial('disk',radius)>0;
    % row = zeros(val_height*val_width,1);
    % col = zeros(val_height*val_width,1);
    % index = 0;
    % for l = 1:val_height
        % for c = 1:val_width
            % val_ref = val(l,c);
            % neigh_val = val_enlarge(l:l+2*radius,c:c+2*radius);
            % neigh_mask = val_mask( l:l+2*radius,c:c+2*radius).*mask;
            % neigh_sort = sort(neigh_val(neigh_mask==1));
            % if val_ref==neigh_sort(end) && val_ref>neigh_sort(end-1)
                % index = index+1;
                % row(index,1) = l;
                % col(index,1) = c;
                % max_local(l,c) = val_ref;
            % end
        % end
    % end
    % row(index+1:end,:) = [];
    % col(index+1:end,:) = [];
end



%--------------------------------------------- RANSAC CODE --------------------------------------
% RANSAC code:
function [final_inliers flag bestmodel] = AffinePairwiseRansac(frames_a1, frames_a2, all_matches)
    % iterations = 0
    % bestfit = nil
    % besterr = something really large
    % while iterations < k {
    % maybeinliers = n randomly selected values from data
    % maybemodel = model parameters fitted to maybeinliers
    % alsoinliers = empty set
    % for every point in data not in maybeinliers {
    % if point fits maybemodel with an error smaller than t
    % add point to alsoinliers
    % }
    % if the number of elements in alsoinliers is > d {
    % this implies that we may have found a good model
    % now test how good it is
    % bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
    % thiserr = a measure of how well model fits these points
    % if thiserr < besterr {
    % bestfit = bettermodel
    % besterr = thiserr

    % }
    % }
    % increment iterations
    % }
    % return bestfit
    %
    %first decide how many matches we have
    MIN_START_VALUES = 4;
    num_matches = size(all_matches, 2);

    if (num_matches < MIN_START_VALUES)
        final_inliers = [];
        bestmodel = [];
        flag = -1;
        return
    end

    % Todo? These might have to be changed if the values are different.
    Z_OFFSET = 640;
    COND_THRESH = 45;
    % RANSAC parameters
    NUM_START_VALUES = 3; % only 3 corrospondences needed for determining affine model
    K = 50;
    ERROR_THRESHOLD = 10; % fairly high threshold - this is in number of pixels
    D = 1; % additional points must fit any given affine model
    N = NUM_START_VALUES;
    RADIUS = 30; %changed to 30 by vijay
    MIN_NUM_OUTSIDE_RADIUS = 1;
    %best error, best fit
    iteration = 0;
    besterror = inf;
    bestmodel = [];
    final_inliers = [];
    max_inliers = 0;

    while (iteration < K)
        %start with NUM_START_VALUES unique values
        uniqueValues = [];
        max_index = size(all_matches, 2);

        while (length(uniqueValues) < NUM_START_VALUES)
            value = ceil(max_index * rand(1, 1));

            if (length(find(value == uniqueValues)) == 0)
                %unique non-zero value

                uniqueValues = [uniqueValues value];
            end

        end

        %uniqueValues are the indices in all_matches
        maybeinliers = all_matches(:, uniqueValues); %start with NUM_START_VALUES unique
        random values
        % make sure points are well distributed
        point_matrix = [frames_a1(:, maybeinliers(1, :)); Z_OFFSET * ones(1, NUM_START_VALUES)];

        if (cond(point_matrix) > COND_THRESH)
            iteration = iteration + 1;
            continue;
        end

        M_maybemodel = getModel(maybeinliers, frames_a1, frames_a2);

        if (prod(size(M_maybemodel)) == 0)
            iteration = iteration + 1;
            continue;
        end

        alsoinliers = [];
        %figure out other inliers
        for i = 1:size(all_matches, 2)
            temp = find(all_matches(1, i) == maybeinliers(1, :));

            if (length(temp) == 0)
                %this means, point not in maybeinlier
                a1 = frames_a1(1:2, all_matches(1, i));
                a2 = frames_a2(1:2, all_matches(2, i));

                if (getError(M_maybemodel, a1, a2) < ERROR_THRESHOLD)
                    alsoinliers = [alsoinliers all_matches(:, i)];
                end

            end

        end

        if (size(alsoinliers, 2) > 0)
            num = 0;
            dist = [];

            for i = 1:NUM_START_VALUES
                diff = frames_a1(1:2, alsoinliers(1, :)) - ...
                    repmat(frames_a1(1:2, maybeinliers(1, i)), [1, size(alsoinliers, 2)]);
                dist = [dist; sqrt(sum(diff.^2))];
            end

            num = sum(sum(dist > RADIUS) == NUM_START_VALUES);

            if (num < MIN_NUM_OUTSIDE_RADIUS)
                iteration = iteration + 1;

                continue;
            end

        end

        %see how good the model is
        %fprintf('Number of elements in also inliers %d\n', size(alsoinliers,2));
        if (size(alsoinliers, 2) > D)
            %this implies that we have found a good model
            %now let's see how good it is
            %find new model
            all_inliers = [maybeinliers alsoinliers];
            M_bettermodel = getModel(all_inliers, frames_a1, frames_a2);
            %the new model could be bad
            if (prod(size(M_bettermodel)) == 0)
                iteration = iteration + 1;
                continue;
            end

            %find error for the model
            thiserror = getModelError(M_bettermodel, all_inliers, frames_a1, frames_a2);

            if max_inliers < size(all_inliers, 2) | (thiserror < besterror & max_inliers == size(all_inliers, 2))
                bestmodel = M_bettermodel;
                besterror = thiserror;
                final_inliers = all_inliers;
                max_inliers = size(final_inliers, 2);
            end

        end

        %do it K times
        iteration = iteration + 1;
    end

    %bestmodel has the best Model
    if (prod(size(bestmodel)) ~= 0)
        % a model was found
        fprintf('Error of best_model ~%f pixels\n', besterror);
        flag = 1;
    else
        flag = -1;
        final_inliers = [];
        bestmodel = [];
        fprintf('No good model found !\n');
    end

end

function error = getModelError(M, matches, frames_a1, frames_a2)
    nummatches = size(matches, 2);
    error = 0;

    for i = 1:nummatches
        a1 = frames_a1(1:2, matches(1, i));
        a2 = frames_a2(1:2, matches(2, i));
        error = error + getError(M, a1, a2);
    end

    error = error / nummatches;
end

function M = getModel(matches, frames_a1, frames_a2)
    %let's go from 1 to 2 -- changed on Apr 28 to be consistent
    %with epipolar and perspective models
    singular_thresh = 1e-6;
    scaling_ratio_thresh = 5;
    scale_thresh = 0.005;
    % approximate M
    M = zeros(3, 3);
    Y = []; X = [];

    for i = 1:size(matches, 2)
        a1 = frames_a1(1:2, matches(1, i));
        a2 = frames_a2(1:2, matches(2, i));
        Y = [Y; a2];
        X = [X; a1(1) a1(2) 1 0 0 0; 0 0 0 a1(1) a1(2) 1];
    end

    %to check if matrix is singular
    if (1 / cond(X) < singular_thresh)
        M = [];
        return
    end

    M = X \ Y;
    %we need to return M - a 3X3 matrix, where the last row is (0 0 1)
    M = [reshape(M, 3, 2)'; 0 0 1];
    %let's add some rules to remove any crazy map
    %we definitely cannot have reflection
    [u, s, v] = svd(M(1:2, 1:2));

    if (det(u * v') < 0)
        %==> there is a reflection
        M = [];

        % fprintf('Special case to avoid reflection\n');
        return
    end

    %we cannot have crazy ratios of scaling in the two dimensions.
    if (cond(M(1:2, 1:2)) > scaling_ratio_thresh)
        %==> the matches are bad
        M = [];
        % fprintf('Special case to avoid crazy scaling ratio\n');
        return
    end

    %check for crazy zoom
    if (s(1, 1) < scale_thresh | s(2, 2) < scale_thresh)
        M = [];
    end

end

function error = getError(M, a1, a2)
    %a2_model is the value of a2 that comes from the model
    %calculate mapping error
    a2_model = M * ([a1; 1]); %3x1 vector, only the first two values matter
    error = dist(a2, a2_model(1:2));
end

function d = dist(one, two)
    d = sqrt(sum((one - two).^2));
end

% function [final_inliers flag bestmodel] = PerspectivePairwiseRansac(frames_a1, frames_a2, all_matches)
%     % iterations = 0
%     % bestfit = nil
%     % besterr = something really large
%     % while iterations < k {
%     % maybeinliers = n randomly selected values from data
%     % maybemodel = model parameters fitted to maybeinliers
%     % alsoinliers = empty set
%     % for every point in data not in maybeinliers {
%     % if point fits maybemodel with an error smaller than t
%     % add point to alsoinliers
%     % }
%     % if the number of elements in alsoinliers is > d {
%     % this implies that we may have found a good model
%     % now test how good it is

%     % bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
%     % thiserr = a measure of how well model fits these points
%     % if thiserr < besterr {
%     % bestfit = bettermodel
%     % besterr = thiserr
%     % }
%     % }
%     % increment iterations
%     % }
%     % return bestfit
%     %
%     %finds the model from the first image to the second image
%     %
%     % [XW] = [a b c][x]
%     % [YW] = [d e f][y]
%     % [W] = [g h 1][1]
%     %(x,y,1) are points in the first image and they map to (XW, YW, W) in
%     %the second image
%     %first decide how many matches we have
%     MIN_START_VALUES = 20;
%     num_matches = size(all_matches, 2);

%     if (num_matches < MIN_START_VALUES)
%         final_inliers = [];
%         bestmodel = [];
%         flag = -1;
%         return
%     end

%     %RANSAC parameters
%     K = 150;
%     NUM_START_VALUES = 4; % using 4 point least squares solution
%     ERROR_THRESHOLD = 10; % this is in the number of pixels.
%     D = 8; %start with 4 points and fit atleast 8 more fit the model
%     %best error, best fit
%     iteration = 0;
%     besterror = inf;
%     bestmodel = [];
%     final_inliers = [];
%     max_inliers = 0;

%     while (iteration < K)
%         %start with NUM_START_VALUES unique values

%         uniqueValues = [];
%         max_index = size(all_matches, 2);

%         while (length(uniqueValues) < NUM_START_VALUES)
%             value = ceil(max_index * rand(1, 1));

%             if (length(find(value == uniqueValues)) == 0)
%                 %unique non-zero value
%                 uniqueValues = [uniqueValues value];
%             end

%         end

%         %uniqueValues are the indices in all_matches
%         maybeinliers = all_matches(:, uniqueValues); %start with NUM_START_VALUES unique
%         random values
%         M_maybemodel = getModel(maybeinliers, frames_a1, frames_a2);

%         if (prod(size(M_maybemodel)) == 0)
%             iteration = iteration + 1;
%             continue;
%         end

%         alsoinliers = [];
%         %figure out other inliers
%         for i = 1:size(all_matches, 2)
%             temp = find(all_matches(1, i) == maybeinliers(1, :));

%             if (length(temp) == 0)
%                 %this means, point not in maybeinlier
%                 a1 = frames_a1(1:2, all_matches(1, i));
%                 a2 = frames_a2(1:2, all_matches(2, i));

%                 if (getError(M_maybemodel, a1, a2) < ERROR_THRESHOLD)
%                     alsoinliers = [alsoinliers all_matches(:, i)];
%                 end

%             end

%         end

%         %see how good the model is
%         %fprintf('Number of elements in also inliers %d\n', size(alsoinliers,2));
%         if (size(alsoinliers, 2) > D)
%             %this implies that we have found a good model
%             %now let's see how good it is
%             %find new model
%             all_inliers = [maybeinliers alsoinliers];
%             M_bettermodel = getModel(all_inliers, frames_a1, frames_a2);
%             %the new model could be bad
%             if (prod(size(M_bettermodel)) == 0)
%                 iteration = iteration + 1;
%                 continue;
%             end

%             %find error for the model
%             thiserror = getModelError(M_bettermodel, all_inliers, frames_a1, frames_a2);

%             if max_inliers < size(all_inliers, 2)|(thiserror<besterror&(max_inliers == size(all_inliers, 2)))
%                 bestmodel = M_bettermodel;
%                 besterror = thiserror;
%                 final_inliers = all_inliers;
%                 max_inliers = size(final_inliers, 2);
%             end
%         end
%         %do it K times
%         iteration = iteration + 1;
%     end

%     if (prod(size(bestmodel)) ~= 0)
%         % a model was found
%         fprintf('Error of best_model ~%f pixels\n', besterror);
%         flag = 1;
%     else
%         flag = -1;
%         final_inliers = [];
%         bestmodel = [];
%         fprintf('No good model found !\n');
%     end

% end


% % function error = getModelError(M, matches, frames_a1, frames_a2)
% %     nummatches = size(matches, 2);
% %     error = 0;

% %     for i = 1:nummatches
% %         a1 = frames_a1(1:2, matches(1, i));
% %         a2 = frames_a2(1:2, matches(2, i));
% %         error = error + getError(M, a1, a2);
% %     end

% %     error = error / nummatches;
% % end

% function P = getModel(matches, frames_a1, frames_a2)
%     %goes from 1 to 2
%     %let's use a least squares approach. Referenced from
%     %http://alumni.media.mit.edu/~cwren/interpolator
%     nummatches = size(matches, 2);
%     LHS = [];

%     for i = 1:nummatches
%         a1 = frames_a1(1:2, matches(1, i)); x = a1(1); y = a1(2);
%         a2 = frames_a2(1:2, matches(2, i)); X = a2(1); Y = a2(2);
%         LHS = [LHS; x y 1 0 0 0 -X * x -X * y; 0 0 0 x y 1 -Y * x -Y * y];
%     end

%     RHS = reshape(frames_a2(1:2, matches(2, :)), nummatches * 2, 1);
%     P = reshape([(LHS \ RHS); 1], 3, 3)';
%     %to get P in the form
%     % [a b c; d e f; g h 1];
% end

% function error = getError(P, a1, a2)
%     %the model F goes from image 1 to image 2
%     %the corresponding point for a1
%     temp = P * [a1; 1];
%     a2_model(1) = temp(1) / temp(3);
%     a2_model(2) = temp(2) / temp(3);
%     error = dist(a2_model', a2);
% end

% function d = dist(one, two)
%     d = sqrt(sum((one - two).^2));
% end
