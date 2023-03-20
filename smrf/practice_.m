% i=imread('coca.jpg');
% i1=rgb2gray(i); %转灰度图像
% figure,imshow(i1);
% i2=im2bw(i1);    %二值化搜索
% figure,imshow(i2);
% i3 = bwmorph(i2,'close');  %闭运算
% figure,imshow(i3);
% i4 = bwmorph(i2,'open');  %开运算
% figure,imshow(i4);
% %bwmorph还支持类似bothat tophat thin等操作个体看下help参数


M=dlmread('F:\3.大二下文档\SRTP文献\滤波标准测试数据15个\samp11.txt');%将有分隔符的数据读成矩阵形式
x = M(:,1);
y = M(:,2);
z = M(:,3);
gobs = M(:,4);  % 0 is Ground, 1 is Object
clear M;

% Declare parameters for this sample (Pingel et al., 2011)
c = 1;  %cellSize
s = .2;  %slopeThreshold
w = 16; %wkmax
et = .45;%elevationThreshold
es = 1.2;%elevationScaler

% Run filter
[ZI R gest] = smrf(x,y,z,'c',c,'s',s,'w',w,'et',et,'es',es);
































