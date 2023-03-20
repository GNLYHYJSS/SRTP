% Test SMRF against ISPRS data set.

% Download reference LIDAR data if not in temp or current dir
%if ~exist([tempdir,'\','samp11.txt']) & ~exist('samp11.txt')
 %   disp('Downloading data.');
  %  url = 'http://www.itc.nl/isprswgIII-3/filtertest/Reference.zip';
   % fn = 'Reference.zip';
    %urlwrite(url,[tempdir,'\',fn]);
    %unzip([tempdir,'\',fn], tempdir);
%end

%%
% Read data
%M = dlmread([tempdir,'']);
M=dlmread('D:\必备资料\SRTP文献\SRTP文献\滤波标准测试数据15个\samp22.txt');%将有分隔符的数据读成矩阵形式
% M1=[];
% [row,~]=size(M);
% for i=1:row
%     if M(i,3)>=340
%         M1=[M1;M(i,:)];
%     end
% end
x = M(:,1);
y = M(:,2);
z = M(:,3);
gobs = M(:,4);  % 0 is Ground, 1 is Object
clear M;
% Declare parameters for this sample (Pingel et al., 2011)
c = 1;  %cellSize
s = .16;  %slopeThreshold
w = 18; %wkmax
et = .35;%elevationThreshold
es = 1.3;%elevationScaler

% Run filter  gest是判断地物是否为地面点0，地物点1 ZI（ZIfin）是最终的DEM
[  ZI ,R, gest] = smrf(x,y,z,'c',c,'s',s,'w',w,'et',et,'es',es);
% gsurfs=reshape(gsurfs,[52119,1]);
% figure(1)
% surf(gsurfs,'edgecolor','none')
% title('格网坡度变化图')
% Report results
ct = crosstab(gobs,gest)%gobs是标准结果、gest是本文计算所得的结果
% 得到的是一个列联表，2*2的矩阵
%% 第一类误差：地面点误判为地物  第二类误差：地物点误判为地面
error1=ct(1,2)/(ct(1,1)+ct(1,2));
error2=ct(2,1)/(ct(2,1)+ct(2,2));
error=(ct(1,2)+ct(2,1))/(ct(1,1)+ct(1,2)+ct(2,1)+ct(2,2));
disp(error1)
disp(error2)
disp(error)
ti = 50;
hfig = figure;
plot3(x,y,z,'.','markersize',4);
axis equal vis3d
% axis([min(x) max(x) min(y) max(y) min(z) max(z)]);
daspect([1 1 1]);
set(gca,'xtick',[min(x):ti:max(x)]);
set(gca,'ytick',[min(y):ti:max(y)]);
set(gca,'ztick',[min(z):ti:max(z)]);
set(gca,'xticklabel',{[0:50:max(x)-min(x)]});
set(gca,'yticklabel',{[0:50:max(y)-min(y)]});
set(gca,'zticklabel',{[0:50:max(z)-min(z)]});
figDPI = '600';
figW = 5;
figH = 5;
grid on
set(gca,'fontsize',8)
set(hfig,'PaperUnits','inches');
set(hfig,'PaperPosition',[0 0 figW figH]);
fileout = ['vcs-smrf-samp11-dotview.'];
print(hfig,[fileout,'tif'],'-r600','-dtiff');
print(hfig,[fileout,'png'],'-r600','-dpng'); 
%%
ti = 50;
hfig = figure;
colormap gray;
[xi yi] = ir2xiyi(ZI,R);
[XI YI] = meshgrid(xi,yi);
surf(XI,YI,ZI,hillshade2(ZI),'edgecolor','none');
% surf(ZI,hillshade2(ZI),'edgecolor','none');
axis equal vis3d;
% axis([min(x) max(x) min(y) max(y) min(z) max(z)]);
daspect([1 1 1]);
set(gca,'xtick',[min(x):ti:max(x)]);
set(gca,'ytick',[min(y):ti:max(y)]);
set(gca,'ztick',[min(z):ti:max(z)]);
set(gca,'xticklabel',{[0:50:max(x)-min(x)]});
set(gca,'yticklabel',{[0:50:max(y)-min(y)]});
set(gca,'zticklabel',{[0:50:max(z)-min(z)]});
view(3);
set(hfig,'PaperUnits','inches');
set(hfig,'PaperPosition',[0 0 figW figH]);
set(hfig,'PaperPosition',[0 0 figW figH]);
set(gca,'fontsize',8)
fileout = ['vcs-smrf-samp11-demview.'];
print(hfig,[fileout,'tif'],'-r600','-dtiff');
print(hfig,[fileout,'png'],'-r600','-dpng'); 
