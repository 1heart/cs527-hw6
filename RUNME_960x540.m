%% Experiment 2

n = 150;

% Do we want to see images and reconstruction errors?
verbose = true;

% Side length of a cubic box
side = 210;

% Camera positions in the frame of reference of the box
distance = 500;
t = distance * [unit([0.9 1 1]); unit([1.1 1 1])]';

% Observations
% Rotation and translation error are 0 regardless
% Structure error and reconstruction error decrease with lower resolution
% Image seems to get narrower with lower resolution
% When noise is added, rotation and translation are non-zero but decrease inversely with resolution
% Sigma can be much higher and the reconstructed image will still be recognizable
% Even looking at relative scale, the errors are higher for the lower resolution images, even though they have significantly less noise

% 960 x 540
[box, camera, img] = world(side, t, 960, 540);

% Compute the true transformation between the camera reference frames
G = camera(2).G / camera(1).G;
% True structure
X = [box(1).X, box(2).X];
% Compute image coordinates in the canonical reference frame
K1 = camera(1).Ks * camera(1).Kf;
K2 = camera(2).Ks * camera(2).Kf;

sigmaVals2 = 0:.25:4; % Breaks down completely at 1.5

eR_Arr = zeros(size(sigmaVals2));
et_Arr = zeros(size(sigmaVals2));
eP_Arr = zeros(size(sigmaVals2));
eImg_Arr = zeros(size(sigmaVals2));

for i = 1:size(sigmaVals2, 2)
  img2 = addNoise(img, sigmaVals2(i));
  % showImages(img2, camera, 2*i - 1);

  x1 = K1 \ [img2(1, 1).x, img2(2, 1).x];
  x2 = K2 \ [img2(1, 2).x, img2(2, 2).x];

  [GComputed, XComputed] = longuetHiggins(x1, x2);
  boxComputed = replaceShape(box, camera(1).G \ XComputed);

  % fig = 2 * i - 1;
  % figure(fig)
  % showStructure(box, 'True Structure');

  fig = 2 * i;
  figure(fig)
  showStructure(boxComputed, 'Reconstructed Structure');

  placeFigures

end

for i = 1:size(sigmaVals2, 2)
  for j = 1:n
    curImg = addNoise(img, sigmaVals2(i));
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
  end
end

eR_Arr = eR_Arr / n;
et_Arr = et_Arr / n;
eP_Arr = eP_Arr / n;
eImg_Arr = eImg_Arr / n;

% Motion error
% Translation and Rotation Error are in degrees, separate plot
figure
plot(sigmaVals2,eR_Arr, sigmaVals2, et_Arr);
title('Graph of Motion Error (n = 150)');
ylabel('Error (degrees)');
xlabel('Sigma value');
legend('Rotation error','Translation error');

% Structure error

figure
plot(sigmaVals2, eP_Arr);
title('Graph of Structure Error (n = 150)');
xlabel('Sigma value')
ylabel('Error (length units per point)');


% Reprojection error
figure
plot(sigmaVals2, eImg_Arr);
title('Graph of Reprojection Error (n = 150)');
xlabel('Sigma value')
ylabel('Error (pixels per point)')

