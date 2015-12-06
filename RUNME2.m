% Simple experiment on reconstruction accuracy using a synthetic world and
% the Longuet-Higgins eight-point algorithm

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

sigmaVals = 0:.5:4;
for i = 1:size(sigmaVals, 2)
  for j = 1:30
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

eR_Arr = eR_Arr / 30;
et_Arr = et_Arr / 30;
eP_Arr = eP_Arr / 30;
eImg_Arr = eImg_Arr / 30;
e1_Arr = e1_Arr / 30;
e2_Arr = e2_Arr / 30;

% Translation and Rotation Error are in degrees, separate plot





% fig = 1;
% if verbose
%     fprintf(1, 'Image noise standard deviation %.2g pixels\n', sigma);
    
%     % Display the images
%     showImages(img, camera, fig);
%     fig = fig + 2;
% end







% % Display reprojection errors
% figure(fig)
% showReprojectionError(e1, e2, x1, x2, camera, img, ...
%     'With the eight-point algorithm');

% % Display true and reconstructed scene structure in world coordinates
% boxComputed = replaceShape(box, camera(1).G \ XComputed);

% fig = fig + 1;
% figure(fig)
% showStructure(box, 'True Structure');

% fig = fig + 1;
% figure(fig)
% showStructure(boxComputed, 'Reconstructed Structure');

% placeFigures
