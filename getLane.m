function [image, goodFrame] = getLane(frame, horizon)

% Get the specific frame

imRGB = im2double(frame);

%imGray = im2double(rgb2gray(imread(file)));
imGray = im2double(rgb2gray(frame));
imGray = imGray(horizon:end, :);
%imshow(im);

src = [0, 720; 1280, 720; 750, horizon; 530, horizon];
src = [src(:, 1), src(:, 2) - horizon];


% Define the region of interest by defining a mask

mask = poly2mask(src(:, 1), src(:, 2), size(imGray, 1), size(imGray, 2));
imGray = imGray .* mask;

% Retrieve the region of interest in the image using the mask

for j = 1:3
    temp(:, :, j) = imRGB(horizon:end, :, j) .* mask;
end
imRGB = temp;

des = [0, 500; 500, 500; 500, 0; 0, 0];

% imshow(im)
% hold on

% Transform the frame to Bird's eye view

% Calculate the transformation function of image
tform = fitgeotrans(src, des, 'projective');
[imNew, xf1_ref] = imwarp(imGray, tform);

% Warp the image to Bird's eye view using the transformation function
for j = 1:3
    [imTransRGB(:, :, j), ~] = imwarp(imRGB(:, :, j), tform);
end

% Threshold the image to keep only the white lane markings

logmask = (imNew > 0.7);
imBird = imNew .* logmask;

% For Detection of Critical Points

windowWidth = 50;
windowHeight = 100;

% Define the window for detection
w = ones(1, windowWidth);

% Find maxmima in the vertical dimension on Left side 

% s=sum(imBird);
%[lol, idx] = max(conv(s, ones(1, 50)));
leftimBird = imBird(ceil(0.6*size(imBird, 1)):end, 1:ceil(0.5*size(imBird, 2)));
[~, Lidx] = max(conv(sum(leftimBird), w));
Lidx = Lidx - windowWidth/2;

% Find maxmima in the vertical dimension on Right side 

rightimBird = imBird(ceil(0.6*size(imBird, 1)):end, ceil(0.5*size(imBird, 2)):end);
[~, Ridx] = max(conv(sum(rightimBird), w));
Ridx = Ridx - windowWidth/2 + size(imBird, 2)/2;
%sum(imBird);

[height, width] = size(imBird);
%midPoints = [Lidx, height, Ridx, height];
midPoints = [Lidx, Ridx];
goodFrame = 1;

% Detect the critical points on the left and right side using a 
% vertical sliding window finding maximas in each case

for i = 1:floor(height / windowHeight) - 1
    splice = sum(imBird(height - (i+1)*windowHeight : height - i*windowHeight, :));
    avg = conv(w, splice);
    delta = windowWidth  / 2;
    leftMin = ceil(max(Lidx + delta - 30, 0));
    leftMax = ceil(min(Lidx + delta + 30, width));
    rightMin = ceil(max(Ridx + delta - 30, 0));
    rightMax = ceil(min(Ridx + delta + 30, width));
    if leftMin <= 0 || rightMin <=0
        goodFrame = 0;
        image = [];
        break;
    end
    [~, Lidx] = max(avg(leftMin:leftMax));
    Lidx = Lidx + leftMin - delta;
    [~, Ridx] = max(avg(rightMin:rightMax));
    Ridx = Ridx + rightMin - delta;
    midPoints = [midPoints; [Lidx, Ridx]];
end

if goodFrame == 1
    midPointY = height+windowHeight/2 : -windowHeight: height - (i+1)*windowHeight;
    midPointY = midPointY(2:end);
    midPointsLeft = [midPoints(:, 1), midPointY'];
    midPointsRight = [midPoints(:, 2), midPointY'];
    % figure;
    % imshow(imBird);

    % Curve fitting through the Critical Points
    leftCurve = ceil(interp1(midPointsLeft(:, 2), midPointsLeft(:, 1), 1:height,'spline'));
    rightCurve = ceil(interp1(midPointsRight(:, 2), midPointsRight(:, 1), 1:height, 'spline'));
    % hold on; plot(leftCurve, 1:height, '-r');
    % hold on; plot(rightCurve, 1:height, '-G');
   
   
	% For Lane Change Detection
    if ~any(leftCurve > rightCurve)
        for j = 1:height
            imTransRGB(j, leftCurve(j):rightCurve(j), 3) = 50;
        end
    else
        for j = 1:height
            imTransRGB(j, leftCurve(j):rightCurve(j), 1) = 50;
        end
    end

	% Transform back from bird's eye view to normal view
    tform2 = fitgeotrans(des, src, 'projective');
    for j = 1:3
        [imReturn(:, :, j), ~] = imwarp(imTransRGB(:,floor(size(imNew,2)/2-250):floor(size(imNew,2)/2+250), j), tform2);
    end
    % figure;
    % imshow(imReturn);
	
	% Return back the  frame with detected lane
    image  = imReturn;
    % figure;

end
end

