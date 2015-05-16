function [distance] = apc_emd(image_1, image_22)

threshold= 10;
alpha_color= 1/2;
coordinates_transformation= 2; 
im1= imresize( image_1 , 1/30);
im2= imresize( image_2 , 1/30);
% 3.jpg is more similar to 1.jpg and thus the running time will be longer.
%im2= imresize( imread('3.jpg') , 1/30);
im1_Y= size(im1,1);
im1_X= size(im1,2);
im1_N= im1_Y*im1_X;
im2_Y= size(im2,1);
im2_X= size(im2,2);
im2_N= im2_Y*im2_X;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EMD input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P= [ ones(im1_N,1)  ;  zeros(im2_N,1) ];
Q= [ zeros(im1_N,1) ;  ones(im2_N,1)  ];

im1= double(im1)./255;
im2= double(im2)./255;
cform = makecform('srgb2lab');
im1_lab= applycform(im1, cform);
im2_lab= applycform(im2, cform);

% Creating the ground distance matrix.
% Loops in Matlab are very time consuming, so I use mex.
% Matlab corresponding code is commented out below.
tic
[ground_distance_matrix]= color_spatial_EMD_ground_distance(im1_lab,im2_lab,...
                                                  alpha_color,threshold, ...
                                                  coordinates_transformation);
fprintf(1,'computing the ground distance matrix, time in seconds: %f\n', ...
        toc);

% In ICCV paper extra_mass_penalty did not matter as image sizes were the same.
extra_mass_penalty= -1;
flowType= 3;

tic
[emd_hat_mex_val_with_flow F]= emd_hat_mex(P,Q,ground_distance_matrix,extra_mass_penalty,flowType);
fprintf(1,'Note that the ground distance here is not a metric.\n');
fprintf(1,'emd_hat_mex, time in seconds: %f\n',toc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

distance = emd_hat_mex_val_with_flow;