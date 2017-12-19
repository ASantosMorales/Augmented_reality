%% Inicialization
close all; clear all; clc;

cd 'Folder where the cameraParams.mat, worldPoints.mat and notsohappyface.jpg where saved'

% In my case I used an extra webcam called Logitech HD Webcam C525
cam = webcam('Logitech HD Webcam C525');

load('cameraParams.mat');
load('worldPoints.mat');

I = imread('notsohappyface.jpg');
I = imresize(rgb2gray(I), [100 100]);
%% Realidad aumentada con imagen precargada

P = [0; 0; 0; 1];
Q = [38; 0; 0; 1];
R = [38; 38; 0; 1];
S = [0; 38; 0; 1];
T = [0; 0; -38; 1];
U = [38; 0; -38; 1];
V = [38; 38; -38; 1];
W = [0; 38; -38; 1];
K = cameraParams.IntrinsicMatrix';
C = zeros(480, 640);

for i=1:500; %Frames number (500 frames is about 1.5 minutes of video)
    Im = snapshot(cam);
    Im_bin = im2bw(rgb2gray(Im), 0.25);
    [imagePoints, boardSize] = detectCheckerboardPoints(rgb2gray(Im));
    if imagePoints > 0
        [rotationMatrix,translationVector] = extrinsics(imagePoints,worldPoints,cameraParams);
        p = P'*[rotationMatrix; translationVector]*K';
        p = p./p(3);
        q = Q'*[rotationMatrix; translationVector]*K';
        q = q./q(3);
        r = R'*[rotationMatrix; translationVector]*K';
        r = r./r(3);
        s = S'*[rotationMatrix; translationVector]*K';
        s = s./s(3);
        t = T'*[rotationMatrix; translationVector]*K';
        t = t./t(3);
        u = U'*[rotationMatrix; translationVector]*K';
        u = u./u(3);
        w = W'*[rotationMatrix; translationVector]*K';
        w = w./w(3);
        projective_xy = fitgeotrans([1 1;  100 1; 100 100; 1 100], [[p(1) p(2)]; [q(1) q(2)]; [r(1) r(2)]; [s(1) s(2)]], 'projective');
        Im_xy = imwarp(I, projective_xy, 'OutputView', imref2d([480 640]), 'FillValues', 150);
        projective_zy = fitgeotrans([1 1;  100 1; 100 100; 1 100], [[w(1) w(2)]; [t(1) t(2)]; [p(1) p(2)]; [s(1) s(2)]], 'projective');
        Im_zy = imwarp(I, projective_zy, 'OutputView', imref2d([480 640]), 'FillValues', 150);
        projective_zx = fitgeotrans([1 1;  100 1; 100 100; 1 100], [[t(1) t(2)]; [u(1) u(2)]; [q(1) q(2)]; [p(1) p(2)]], 'projective');
        Im_zx = imwarp(I, projective_zx, 'OutputView', imref2d([480 640]), 'FillValues', 150);
        C_1 = imfuse(Im, Im_xy, 'blend','Scaling','independent');
        C_2 = imfuse(Im_zy, Im_zx, 'blend','Scaling','independent');
        C = imfuse(C_1, C_2, 'blend','Scaling','independent');
    end
    imshow(C);
    hold on
end
