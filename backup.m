
clear
close all

laser = load('/Users/admin/SMR_common_folder/laser_0.log');
odometry = load('/Users/admin/SMR_common_folder/odoPose.log');
odometry = odometry(4:end-4,:);

sync_idx_l = find(abs(laser(:,1)-odometry(1,1))<0.05);
sync_idx_h = find(abs(laser(:,1)-odometry(end,1))<0.05);
laser = laser(sync_idx_l:sync_idx_h,:);
laser(laser<=0.02 & laser>0)= 4;
sync_idx = zeros(size(laser,1),1);
for i=1:size(laser,1)
    sync_idx(i) = find(abs(odometry(:,1)-laser(i,1))<0.05,1);
end

odometry = odometry(sync_idx,:);
angles = laser(1,4):laser(1,3):(laser(1,4)+(size(laser,2)-6)*laser(1,3));
laserx = laser(1:end,6:end).*cosd(angles+rad2deg(odometry(:,4)));
lasery = laser(1:end,6:end).*sind(angles+rad2deg(odometry(:,4)));
laserr = sqrt(laserx.^2 + lasery.^2);
odox = odometry(:,2);
odoy = odometry(:,3);
odoth = odometry(:,4);
plot(laserx(1,:),lasery(1,:),'LineStyle','none','Marker','x');

resolution = 1/80;% 1cm
odox = round(odox,round(log10(1/resolution)));
odoy = round(odoy,round(log10(1/resolution)));

[X,Y] = meshgrid(-.5:resolution:6, 1.5:-resolution:-4.5);

Xr = round(X,round(log10(1/resolution)));
Yr = round(Y,round(log10(1/resolution)));

map = zeros(size(X));

    

laser_sample = 1;
odo_sample = 1;
attenuation = 0.005*odo_sample; %db ,has to be increased when sampling,less occurences

xi=odox(1);
yi=odoy(1);
wheelbase_tolaser=0.26;%distance from wheelbase to laser
laser_pose = [odox odoy]+wheelbase_tolaser*[cos(odoth) sin(odoth)];
dispstyle = 'prob';
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
    
    
    switch(dispstyle)
        case 'prob'
        % Show propab map
        
        map(LoV)= map(LoV)-attenuation; 
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
    
    xx=round((abs(odox(i)-X(end,end)))/resolution);
    yy=round((abs(odoy(i)-Y(1,1)))/resolution);
    
    xi=[xi xx]; %Store previous pose
    yi=[yi yy]; %Store ..
    
    %Plot pose
    hold on
    
    plot(yi,xi,'.r','MarkerSize',8)
    
    plotcrossmarker(odoth(i), 50, yy, xx)
    
    hold off
    drawnow

end

% 
% testx = repmat(X,1,1,size(laserr,2));
% testy = repmat(Y,1,1,size(laserr,2));
% laser_test=reshape(repelem(laserr(i,:),size(X,1),size(X,2)),size(X,1),size(X,2),size(laserr,2));
% angles_test=reshape(repelem(angles,size(X,1),size(X,2)),size(X,1),size(X,2),size(angles,2));
% 
% kk = (((testy-odoy(i)).^2 + (testx-odox(i)).^2)<=laser_test.^2 & abs(atan2(testy-odoy(i),testx-odox(i))-deg2rad(angles_test+rad2deg(odoth(i))))<=laser_sample*deg2rad(abs(laser(1,3)))) ;

function plotcrossmarker(alpha, r, x0, y0)
if nargin < 1, alpha = 0; end;
if nargin < 2, r = 1; end
if nargin < 3, x0 = 0; end
if nargin < 4, y0 = 0; end

points = [[-.13 0 .13 -.13]' [0 -0.26 0 0]'];
points = points*[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)]*r+[x0 y0];

line(points(:,1),points(:,2),'LineWidth',1,'Color','white')
end


