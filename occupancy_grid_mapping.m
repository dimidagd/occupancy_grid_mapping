% Occupancy Grid mapping assuming perfect knowledge of state. 
% All script dependencies are in the same directory.
clear
clc
close all

%Load measurements
laser = load('laser_0.log');
%Define initial pose
initial_pose=[4 -2.7 deg2rad(0.7)];
%Pose data at 100hz, laser data at 10hz. Sync method by colleague.
syncmethod='adam'

switch(syncmethod)
    case('dimitris')
%% Dimitris sync method
odometry = load('/Users/admin/SMR_common_folder/odoPose.log');
odometry = odometry(4:end-4,:);
odometry(:,2:4)=odometry(:,2:4)+initial_pose;

sync_idx_l = find(abs(laser(:,1)-odometry(1,1))<0.05);
sync_idx_h = find(abs(laser(:,1)-odometry(end,1))<0.05);
laser = laser(sync_idx_l:sync_idx_h,:);
laser(laser<=0.02 & laser>0)= 4;
sync_idx = zeros(size(laser,1),1);
for i=1:size(laser,1)
    sync_idx(i) = find(abs(odometry(:,1)-laser(i,1))<0.05,1);
end

odometry = odometry(sync_idx,:);
odox = odometry(:,2);
odoy = odometry(:,3);
odoth = odometry(:,4);
    case('adam')
% Adam sync method
    
odometry = OdometryEstimator(laser(:,1));

odox = [odometry(:).X]'+initial_pose(1);
odoy = [odometry(:).Y]'+initial_pose(2);
odoth = [odometry(:).T]'+initial_pose(3);

laser = laser(~isnan(odox),:);
laser(laser<=0.02 & laser>0)= 4;
odox = odox(~isnan(odox));
odoy = odoy(~isnan(odoy));
odoth = odoth(~isnan(odoth));

%%
end

%Measurements
angles = laser(1,4):laser(1,3):(laser(1,4)+(size(laser,2)-6)*laser(1,3));
laserx = laser(1:end,6:end).*cosd(angles+rad2deg(odoth));
lasery = laser(1:end,6:end).*sind(angles+rad2deg(odoth));
laserr = sqrt(laserx.^2 + lasery.^2);

% plot(laserx(1,:),lasery(1,:),'LineStyle','none','Marker','x');

%Define Grid resolution
resolution = 0.01;
%odox = round(odox,round(log10(1/resolution)));
%odoy = round(odoy,round(log10(1/resolution)));

%Define occupancy grid
[X,Y] = meshgrid(-.5:resolution:6.5, 1.2:-resolution:-4.5);


%Map a default value to all cells (db)
map = 0.1*ones(size(X));

    
%Sample on measurement or keep all. (1=all)
laser_sample = 1;
odo_sample = 1;
%Attenuation of cell when ray casting
attenuation = 3*0.005*odo_sample; % has to be increased when sampling, becaue less occurences on every cell

xi=odox(1);
yi=odoy(1);
wheelbase_tolaser=0.26;%distance from wheelbase to laser, used in transformation from laser to robot pose
laser_pose = [odox odoy]+wheelbase_tolaser*[cos(odoth) sin(odoth)]; %transformation
last_norm=0;
dispstyle = 'prob'; %case argument for different plot styles, alternative dispstyle='snaps'
h = figure;

%Raycasting on all measurements
for i=1:odo_sample:size(laserr,1)-1
    
    %Scanning for each sensor measurement (slow)
%     LoV = zeros(size(map));
%     for k=1:laser_sample:size(laserr,2)
%        LoV = LoV | (((Y-odoy(i)).^2 + (X-odox(i)).^2)<=laserr(i,k)^2 & abs(atan2(Y-odoy(i),X-odox(i))-deg2rad(angles(k)+rad2deg(odoth(i))))<=laser_sample*deg2rad(abs(laser(1,3)))) ;
%     end
    

    %Finding inner area of polygon formed by measurements
    %(fast)
    [stat,~] = inpoly2([X(:)-laser_pose(i,1) Y(:)-laser_pose(i,2)],[laserx(i,:)' lasery(i,:)']);
    LoV = reshape(stat,size(X));
    
    %% State change detection using pose norm,
    %when robot is still do not update map with the same attenuation.
    new_norm = norm(laser_pose(i,:));
    if (abs(new_norm-last_norm)<0.01) %%reducing attenuation for still captures
        att = attenuation/10;
    else
        att = attenuation;
    end
    last_norm = new_norm;
    switch(dispstyle)
        case 'prob'
        % Show propab map
        
        map(LoV)= map(LoV)-att; 
        %a = flip((-10.^(map)+1)');
        a=(-10.^(map)+1);
        imshow(a,[0 1])
        
        case 'snaps'
        %Show snaps
        map(LoV)=1;
        a = flip((map)');
        imshow(a,[0 1])
        map = zeros(size(X));
    end
    ax = gca;
    
    %Debugging code
   % x=X(:)-odox(i);
   % y=Y(:)-odoy(i);
    %plot(x(~stat)-odox(i),y(~stat)-odoy(i),'r.', ...
    %'markersize',14) ;
    
    %plot(laserx(i,:)+odox(i),lasery(i,:)+odoy(i),'b.')
    %xlim([-5 5])
    %ylim([-5 5])
    
    %Pose plot
    %[yy,xx]= find(flip(Xr,2)==odox(i) & Yr==odoy(i),1);
    
    %Make odometry value match a cell value.
    xx=round((abs(odox(i)-X(1,1)))/resolution);
    yy=round((abs(odoy(i)-Y(1,1)))/resolution);
    
    xi=[xi xx]; %Store previous pose
    yi=[yi yy]; %Store ..
    
    %Plot pose
    hold on
    
    trace=plot(xi(1:7:end),yi(1:7:end),'.','MarkerEdgeColor',[0.5 0.5 0.5],'MarkerSize',6);
    %alpha(trace,.1)
    
    plotcrossmarker(odoth(i), 1/resolution, xx, yy)
    
    hold off
    camroll(90)
    drawnow
%% Capture GIF
    frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(imind,cm,'gifloop.gif','gif','DelayTime',0.001, 'Loopcount',inf); 
      elseif (~mod(i,3))
          imwrite(imind,cm,'gifloop.gif','gif','DelayTime',0.001,'WriteMode','append'); 
      end 
    
        
end

% 
% testx = repmat(X,1,1,size(laserr,2));
% testy = repmat(Y,1,1,size(laserr,2));
% laser_test=reshape(repelem(laserr(i,:),size(X,1),size(X,2)),size(X,1),size(X,2),size(laserr,2));
% angles_test=reshape(repelem(angles,size(X,1),size(X,2)),size(X,1),size(X,2),size(angles,2));
% 
% kk = (((testy-odoy(i)).^2 + (testx-odox(i)).^2)<=laser_test.^2 & abs(atan2(testy-odoy(i),testx-odox(i))-deg2rad(angles_test+rad2deg(odoth(i))))<=laser_sample*deg2rad(abs(laser(1,3)))) ;

%% Functions for robot animation and path plotting.
function plotcrossmarker(alph, r, x0, y0)
if nargin < 1, alph = 0; end;
if nargin < 2, r = 1; end
if nargin < 3, x0 = 0; end
if nargin < 4, y0 = 0; end
alph=alph-pi/2;
a=.13;
b=0.28;
points = [[-a -a  a a -a]' [0 -b -b 0 0]'];
points = points*[cos(alph) -sin(alph);sin(alph) cos(alph)]*r+[x0 y0];

wheelsl = ([0.03*[-1 -1 1 1 -1]' 0.1*[.5 -.5 -.5 .5 .5]']+[a -0.05]);
wheelsr = ([0.03*[-1 -1 1 1 -1]' 0.1*[.5 -.5 -.5 .5 .5]']+[-a -0.05]);

wheels=[wheelsl ; wheelsr]*[cos(alph) -sin(alph);sin(alph) cos(alph)]*r+[x0 y0];
h=fill(wheels(1:5,1),wheels(1:5,2),[1 0 0],wheels(6:10,1),wheels(6:10,2),[1 0 0],...
    points(:,1),points(:,2),[0.5 0.8 0.5]-0.15);
%alpha(h,.5)
set(h,'EdgeColor','none')

%line(points(:,1),points(:,2),'LineWidth',1.5,'Color','green')
end


