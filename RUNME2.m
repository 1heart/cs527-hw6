% Simple experiment on reconstruction accuracy using a synthetic world and
% the Longuet-Higgins eight-point algorithm

% TRIALS
n = 150;
% Max sigma
maxSigma = 8;
% Step size
stepSize = 0.25;

% Do we want to see images and reconstruction errors?
verbose = true;

% Side length of a cubic box
side = 210;

% Camera positions in the frame of reference of the box
distance = 500;
t = distance * [unit([0.9 1 1]); unit([1.1 1 1])]';

% Make a 3D box, two cameras, and the resulting images
[box, camera, img] = world(side, t);

% Add zero-mean Gaussian noise to the images
sigma = 0.0;  % Standard deviation of noise, in pixels
img = addNoise(img, sigma);

% Compute the true transformation between the camera reference frames
G = camera(2).G / camera(1).G;
% True structure
X = [box(1).X, box(2).X];
% Compute image coordinates in the canonical reference frame
K1 = camera(1).Ks * camera(1).Kf;
K2 = camera(2).Ks * camera(2).Kf;


sigmaVals = 0:stepSize:maxSigma;


imgArr = {};
% Rotation
eR_Arr = zeros(size(sigmaVals));
% Translation
et_Arr = zeros(size(sigmaVals));
% Structure Errors
eP_Arr = zeros(size(sigmaVals));
% Reprojection Errors
eImg_Arr = zeros(size(sigmaVals));
e1_Arr = zeros(size(sigmaVals));
e2_Arr = zeros(size(sigmaVals));

% sigmaVals2 = 0:1:maxSigma;
% for i = 1:size(sigmaVals2, 2)
%   img2 = addNoise(img, sigmaVals2(i));
%   % showImages(img2, camera, 2*i - 1);

%   x1 = K1 \ [img2(1, 1).x, img2(2, 1).x];
%   x2 = K2 \ [img2(1, 2).x, img2(2, 2).x];

%   [GComputed, XComputed] = longuetHiggins(x1, x2);
%   boxComputed = replaceShape(box, camera(1).G \ XComputed);

%   % fig = 2 * i - 1;
%   % figure(fig)
%   % showStructure(box, 'True Structure');

%   fig = 2 * i;
%   figure(fig)
%   showStructure(boxComputed, 'Reconstructed Structure');

%   placeFigures

% end



for i = 1:size(sigmaVals, 2)
  for j = 1:n
    curImg = addNoise(img, sigmaVals(i));
    imgArr{i} = curImg;
    x1 = K1 \ [curImg(1, 1).x, curImg(2, 1).x];
    x2 = K2 \ [curImg(1, 2).x, curImg(2, 2).x];
    % Compute the transformation between the reference systems of the two
    % cameras and the scene structure in the first camera reference system,
    % using the eight-point algorithm
    [GComputed, XComputed] = longuetHiggins(x1, x2);

    % Measure and report errors before bundle adjustment
    % fprintf(1, '\nAfter running the eight-point algorithm:\n');
    [eR, et] = motionError(GComputed, G, verbose);

    eR_Arr(i) = eR_Arr(i) + eR;
    et_Arr(i) = et_Arr(i) + et;

    eP_Arr(i) = eP_Arr(i) + structureError(XComputed, X, verbose);
    [eImg, e1, e2] = reprojectionError(GComputed, XComputed, ...
        x1, x2, camera, verbose);
    eImg_Arr(i) = eImg_Arr(i) + eImg;
    % e1_Arr(i) = e1_Arr(i) + e1;
    % e2_Arr(i) = e2_Arr(i) + e2;
  end
end

eR_Arr = eR_Arr / n;
et_Arr = et_Arr / n;
eP_Arr = eP_Arr / n;
eImg_Arr = eImg_Arr / n;
e1_Arr = e1_Arr / n;
e2_Arr = e2_Arr / n;

nString = strcat('(n = ', int2str(n),  ' )');

figure
% Motion error
% Translation and Rotation Error are in degrees, separate plot
subplot(2, 2, 1)
plot(sigmaVals,eR_Arr);
title(strcat('Graph of Rotation Motion Error ', nString));
ylabel('Error (degrees)');
xlabel('Sigma value');
% legend('Rotation error','Translation error');

subplot(2, 2, 2)
plot(sigmaVals,et_Arr);
title(strcat('Graph of Translational Motion Error ', nString));
ylabel('Error (degrees)');
xlabel('Sigma value');
% legend('Rotation error','Translation error');

subplot(2, 2, 3)
% Structure error
plot(sigmaVals, eP_Arr);
title(strcat('Graph of Structure Error ', nString));
xlabel('Sigma value')
ylabel('Error (length units per point)');

subplot(2, 2, 4)
% Reprojection error
plot(sigmaVals, eImg_Arr);
title(strcat('Graph of Reprojection Error ', nString));
xlabel('Sigma value')
ylabel('Error (pixels per point)')
