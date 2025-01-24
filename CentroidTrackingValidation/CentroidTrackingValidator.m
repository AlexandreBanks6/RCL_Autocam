clear
clc
close all
%% Init Constants
EXTERNAL_WIDTH=round(0.4*1280); %Width of camera
EXTERNAL_HEIGHT=0.4*960; %Height of camera
MID_WIDTH=EXTERNAL_WIDTH/2;
MID_HEIGHT=EXTERNAL_HEIGHT/2;

%% Reading in and sorting centroid data
data_one=xlsread("CentroidTrackingValidationData\Data_1.csv");
data_two=xlsread("CentroidTrackingValidationData\Data_2.csv");
data_one(1:5,:)=[];
data_two(1:5,:)=[];
data=[data_one;data_two];
%Extracting u,v coordinates of centroid for left/right cam
u_left=data(:,30);
v_left=data(:,31);

u_right=data(:,60);
v_right=data(:,61);

%Getting rid of -1's
u_left=u_left(u_left~=-1);
v_left=v_left(v_left~=-1);
u_right=u_right(u_right~=-1);
v_right=v_right(v_right~=-1);

%% Statistics

%%%%%%%%Left Camera%%%%%%%
%%Euclidean Norm
left_norm_err=sqrt((MID_WIDTH-u_left).^2+(MID_HEIGHT-v_left).^2);
left_norm_avg=mean(left_norm_err);
left_norm_std=std(left_norm_err);

%Large outliers (>2*std) because of error in visual tracking
[left_norm_err_new,left_keep]=rm_outliers(left_norm_err,left_norm_avg,left_norm_std);
left_norm_avg_new=mean(left_norm_err_new);
left_norm_std_new=std(left_norm_err_new);

%%Component Errors
u_left=u_left(left_keep);
left_u_err=abs(MID_WIDTH-u_left);
left_u_avg=mean(left_u_err);
left_u_std=std(left_u_err);

v_left=v_left(left_keep);
left_v_err=abs(MID_HEIGHT-v_left);
left_v_avg=mean(left_v_err);
left_v_std=std(left_v_err);

%%%%%%%%Right Camera%%%%%%%
%%Euclidean Norm
right_norm_err=sqrt((MID_WIDTH-u_right).^2+(MID_HEIGHT-v_right).^2);
right_norm_avg=mean(right_norm_err);
right_norm_std=std(right_norm_err);

%Large outliers (>2*std) because of error in visual tracking
[right_norm_err_new,right_keep]=rm_outliers(right_norm_err,right_norm_avg,right_norm_std);
right_norm_avg_new=mean(right_norm_err_new);
right_norm_std_new=std(right_norm_err_new);

%%Component Errors
u_right=u_right(right_keep);
right_u_err=abs(MID_WIDTH-u_right);
right_u_avg=mean(right_u_err);
right_u_std=std(right_u_err);

v_right=v_right(right_keep);
right_v_err=abs(MID_HEIGHT-v_right);
right_v_avg=mean(right_v_err);
right_v_std=std(right_v_err);

%% Plotting Results

%%%%%%%%Heat Map of u,v%%%%%%%%%%%%
%%%%%%left
figure;
scatplot(u_left,v_left,'circles',2,100,5,3,4);


%%%%%right
%%%%%%left
figure;
scatplot(u_right,v_right,'circles',2,100,5,3,4);

%%%%%%left
figure;
plot(u_left,v_left,'ob');
xlim([0,EXTERNAL_WIDTH]);
ylim([0,EXTERNAL_HEIGHT]);


% Create a 2D histogram for density
num_bins = 50; % Adjust the number of bins for resolution
edges_x = linspace(0, EXTERNAL_WIDTH, num_bins);
edges_y = linspace(0, EXTERNAL_HEIGHT, num_bins);

% Compute the 2D histogram
[N, edges] = histcounts2(u_left, v_left, edges_x, edges_y);

% Plot the heatmap
figure;
imagesc(edges_x, edges_y, N'); % Transpose `N` for proper orientation
set(gca, 'YDir', 'normal'); % Flip the y-axis to match standard plot orientation
colormap('parula'); % Use a 'hot' colormap for density
colorbar; % Show the colorbar
hold on;

% Overlay the scatter plot
%plot(u_left, v_left, 'ob', 'MarkerSize', 3, 'MarkerFaceColor', 'blue');

% Set axis limits
xlim([0, EXTERNAL_WIDTH]);
ylim([0, EXTERNAL_HEIGHT]);
title('Scatter Density Plot');
xlabel('u\_left');
ylabel('v\_left');



%% Functions
function [data_new,keep_indeces]=rm_outliers(data,avg,std)
keep_indeces=data<(avg+2*std) & data>(avg-2*std);
data_new=data(keep_indeces);
end






