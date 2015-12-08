% Simple experiment on reconstruction accuracy using a synthetic world and
% the Longuet-Higgins eight-point algorithm

% Do we want to see images and reconstruction errors?
verbose = true;

% Side length of a cubic box
side = 210;

% Camera positions in the frame of reference of the box
distance = 500;
t = distance * [unit([0.9 1 1]); unit([1.1 1 1])]';

n = 1000;

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

curImg = img;

eR_tot = 0;
et_tot = 0;
eP_tot = 0;
eImg_tot = 0;

for j = 1:n
  curImg(1, 1).x = mod(randn(size(img(1, 1).x)), 5);
  curImg(1, 2).x = mod(randn(size(img(1, 2).x)), 5);
  curImg(2, 1).x = mod(randn(size(img(2, 1).x)), 5);
  curImg(2, 2).x = mod(randn(size(img(2, 2).x)), 5);
  x1 = K1 \ [curImg(1, 1).x, curImg(2, 1).x];
  x2 = K2 \ [curImg(1, 2).x, curImg(2, 2).x];
  % Compute the transformation between the reference systems of the two
  % cameras and the scene structure in the first camera reference system,
  % using the eight-point algorithm
  [GComputed, XComputed] = longuetHiggins(x1, x2);

  % Measure and report errors before bundle adjustment
  % fprintf(1, '\nAfter running the eight-point algorithm:\n');
  [eR, et] = motionError(GComputed, G, verbose);
  
  eR_tot = eR_tot + eR;
  et_tot = et_tot + et;

  eP_tot = eP_tot + structureError(XComputed, X, verbose);

  [eImg, e1, e2] = reprojectionError(GComputed, XComputed, ...
      x1, x2, camera, verbose);
  eImg_tot = eImg_tot + eImg;
end

eR_tot = eR_tot / n
et_tot = et_tot / n
eP_tot = eP_tot / n
eImg_tot = eImg_tot / n