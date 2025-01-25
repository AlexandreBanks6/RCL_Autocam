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

%Reads in time data
data_one_t=readtable("CentroidTrackingValidationData\Data_1.csv");
data_two_t=readtable("CentroidTrackingValidationData\Data_2.csv");
% Step 2: Extract the first column containing timestamps
timestamps_one = data_one_t{:, 1};
timestamps_two = data_two_t{:, 1};
% Convert to strings (if not already in string format)
timestamps_one = string(timestamps_one);
timestamps_two = string(timestamps_two);

time_format = 'HH:mm:ss.SSSSSS'; % Define the timestamp format
datetime_array_one = datetime(timestamps_one, 'InputFormat', time_format);
elapsed_time_one = seconds(datetime_array_one - datetime_array_one(1));
datetime_array_two = datetime(timestamps_two, 'InputFormat', time_format);
elapsed_time_two = seconds(datetime_array_two - datetime_array_two(1));

elapsed_time=[elapsed_time_one;elapsed_time_one(end)+0.001+elapsed_time_two];

%Extracting u,v coordinates of centroid for left/right cam
u_left=data(:,30);
v_left=data(:,31);

u_right=data(:,60);
v_right=data(:,61);

%Corrects time vector
elapsed_time_left=elapsed_time(u_left~=-1);
elapsed_time_right=elapsed_time(u_right~=-1);

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
%[left_norm_err_new,left_keep]=rm_outliers_new(left_norm_err,elapsed_time_left);
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
%[right_norm_err_new,right_keep]=rm_outliers_new(right_norm_err,elapsed_time_right);
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

%%Formats the time arrays and concatenates them
elapsed_time_left=elapsed_time_left(left_keep);
elapsed_time_right=elapsed_time_right(right_keep);
%% Plotting Results

%%%%%%%%Heat Map of u,v%%%%%%%%%%%%
%%%%%%left
fig=figure;
%scatplot(x,y,method,radius,N,n,po,ms)
scatplot(u_left,v_left,'circles',15,100,5,3,15);
hold on
plot(MID_WIDTH,MID_HEIGHT,'xk','MarkerSize',20,'LineWidth',3);
hold off
%set(gca, 'YDir', 'normal'); % Flip the y-axis to match standard plot orientation
xlim([0, EXTERNAL_WIDTH]);
ylim([0, EXTERNAL_HEIGHT]);
title('Left Camera Centroid Tracking Scatter Density Plot','FontSize',12,'FontName','Times','FontWeight','bold');
xlabel('u_{left}','FontSize',12,'FontName','Times');
ylabel('v_{left}','FontSize',12,'FontName','Times');
fig.Position = [100, 100, 800, 600];


%%%%%right
%%%%%%left
fig=figure;
scatplot(u_right,v_right,'circles',15,100,5,3,15);
hold on
plot(MID_WIDTH,MID_HEIGHT,'xk','MarkerSize',20,'LineWidth',3);
hold off
%set(gca, 'YDir', 'normal'); % Flip the y-axis to match standard plot orientation
xlim([0, EXTERNAL_WIDTH]);
ylim([0, EXTERNAL_HEIGHT]);
title('Right Camera Centroid Tracking Scatter Density Plot','FontSize',12,'FontName','Times','FontWeight','bold');
xlabel('u_{right}','FontSize',12,'FontName','Times');
ylabel('v_{right}','FontSize',12,'FontName','Times');
fig.Position = [100, 100, 800, 600];

%%%%%%% u&v error as function of time %%%%%%%
max_err=max([left_u_err;left_v_err;right_u_err;right_v_err]);

%u left error
fig=figure;
plot(elapsed_time_left,left_u_err,'.-','Color','#17bfb6','LineWidth',1);
ylim([0,max_err]);
title('Left Camera U-Coordinate Error','FontSize',12,'FontName','Times','FontWeight','bold');
xlabel('Trial Time (s)','FontSize',12,'FontName','Times');
ylabel('|Screen_{w}/2-u_{left}|','FontSize',12,'FontName','Times');
fig.Position = [100, 100, 1400, 650];

%v left error
fig=figure;
plot(elapsed_time_left,left_v_err,'.-','Color','#fec636','LineWidth',1);
ylim([0,max_err]);
title('Left Camera V-Coordinate Error','FontSize',12,'FontName','Times','FontWeight','bold');
xlabel('Trial Time (s)','FontSize',12,'FontName','Times');
ylabel('|Screen_{h}/2-v_{left}|','FontSize',12,'FontName','Times');
fig.Position = [100, 100, 1400, 650];

%u right error
fig=figure;
plot(elapsed_time_right,right_u_err,'.-','Color','#17bfb6','LineWidth',1);
ylim([0,max_err]);
title('Right Camera U-Coordinate Error','FontSize',12,'FontName','Times','FontWeight','bold');
xlabel('Trial Time (s)','FontSize',12,'FontName','Times');
ylabel('|Screen_{w}/2-u_{right}|','FontSize',12,'FontName','Times');
fig.Position = [100, 100, 1400, 650];

%v right error
fig=figure;
plot(elapsed_time_right,right_v_err,'.-','Color','#fec636','LineWidth',1);
ylim([0,max_err]);
title('Right Camera V-Coordinate Error','FontSize',12,'FontName','Times','FontWeight','bold');
xlabel('Trial Time (s)','FontSize',12,'FontName','Times');
ylabel('|Screen_{h}/2-v_{right}|','FontSize',12,'FontName','Times');
fig.Position = [100, 100, 1400, 650];




%% Functions
function [data_new,keep_indeces]=rm_outliers(data,avg,std)
keep_indeces=data<(avg+2*std) & data>(avg-2*std);
data_new=data(keep_indeces);
end

function [data_new,keep_indeces]=rm_outliers_new(data,sample_points)
[data_new,rej_ind]=rmoutliers(data,"movmean",0.5,"SamplePoints",sample_points);
keep_indeces=~rej_ind;
end






