% progressiveFilter 渐进式滤波
%   Removes high points from a Digital Surface Model by progressive
%   morphological filtering.
%
% Syntax
%   function [isObjectCell] = progressiveFilter(ZI,varargin)
%
%
% Description 渐进式滤波方法是SMRF的心脏
%   The progressive morphological filter is the heart of the SMRF lidar
%   ground filtering package.  Given an image, a cellsize, a slope
%   threshold, and a maximum window size, 
%   
%   the filter uses an image opening
%   operation of iteratively increasing size to locate all "objects" - high
%   points in the image like trees or buildings.
%
%   The progressiveFilter must be minimally called with an image (a digital 
%   surface model) as well as a cellsize (c), a slope threshold value (s), 
%   and a maximum window size (w). 
%   渐进式滤波，至少需要初始DSM、格网大小c、坡度阈值s、最大窗口尺寸w 参数
%   The slope threshold value governs the
%   identification process, and roughly corresponds to the maximum slope of
%   the terrain you are working with.  The maximum window size defines a
%   window radius (in map units), and corresponds to the size of largest 
%   feature to be removed.
%
%
% Requirements
%   Image Processing Toolbox
%   progressiveFilter requires John D'Errico's inpaint_nans.m file.
%   http://mathworks.com/matlabcentral/fileexchange/4551
%
%
% Input Parameters
%   ZI        - A digital surface model, preferably a minimum surface preferably最好是
%
%   'c',c     - Cell size, in map units 本文为1
%
%   's',s     - Defines the maximum expected slope of the ground surface
%               Values are given in dz/dx, so most slope values will be in
%               the range of .05 to .30.
%               本文为0.2
%   'w',w     - Defines the filter's maximum window radius.  最大窗口半径，本文是16
%   'w',[0 w]   Alternatively, the user can supply his or her own vector of
%   'w',[1:5:w] window sizes to control the open process.  
%    也可以自己选择要将窗口尺寸如何变化
%   'inpaintMethod',ipm
%               If this parameter is supplied, it controls the argument
%               passed to D'Errico's inpaint_nans method.  The default
%               value is 4. 内插的方法，4代表Springs的方法
%
%   'cutNet',netSize
%               Cuts a net of spacing netSize (map coordinates) into ZI
%               before further processing.  
%               This can help to remove large
%               buildings without the need for extremely large filter
%               windows. 这样操作之后，不需要使用更大的滤波窗口去去除建筑物
%               Generally, netSize should be set to the largest
%               window radius used (w 16).
%
%
% Output Parameters
%
%   isObjectCell
%               A logical image mask indicating cells flagged as objects.
%
%
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met:
% 
%     * Redistributions of source code must retain the above copyright 
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright 
%       notice, this list of conditions and the following disclaimer in 
%       the documentation and/or other materials provided with the distribution
%       
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE.
    


function [isObjectCell lastSurface thisSurface] = progressiveFilter(lastSurface,varargin) % varargin是可变参数列表
M=dlmread('D:\必备资料\SRTP文献\SRTP文献\滤波标准测试数据15个\samp12.txt');%将有分隔符的数据读成矩阵形式
x = M(:,1);
y = M(:,2);
gobs = M(:,4);
density=length(gobs)/((max(x)-min(x))*(max(y)-min(y)));
clear M;
% Define required input parameters
cellSize = [];
slopeThreshold = [];
wkmax = [];

% Define optional input parameters
inpaintMethod = [];
strelShape = [];

% Define output parameters
isObjectCell = [];


%% Process supplied arguments

i = 1;
while i<=length(varargin)    
    if isstr(varargin{i})
        switchstr = lower(varargin{i}); % varargin是元胞数组
        switch switchstr
            case 'c'
                cellSize = varargin{i+1};
                i = i + 2;
            case 's'
                slopeThreshold = varargin{i+1};
                i = i + 2;
            case 'w'
                wkmax = varargin{i+1};
                i = i + 2;  
            case 'inpaintmethod'
                inpaintMethod = varargin{i+1};
                i = i + 2;
            case 'shape'
                strelShape = varargin{i+1};
                i = i + 2;
            otherwise
                i = i + 1;
        end
    else
        i = i + 1;
    end
end


%% Catch some errors

if isempty(cellSize)
    error('Cell size must be specified.');
end
if isempty(wkmax)
    error('Maximum window size must be specified');
end
if isempty(slopeThreshold)
    error('Slope threshold value must be specified.');
end


%% Define some default parameters

if isempty(inpaintMethod)
    inpaintMethod = 4; % Springs
end

if isempty(strelShape)
    strelShape = 'disk';%disk圆盘，开运算的结构元素形状
end


%% Convert wkmax to a vector of window sizes (radii) defined in pixels.  将最大窗口尺寸在现实中的坐标转换到像素坐标系下面
% If w was supplied as a vector, use those values as the basis; otherwise,
% use 1:1:wkmax

if numel(wkmax)~=1   %numel函数（元素个数） 和 ceil函数（向上取整）
    wk = ceil(wkmax / cellSize);
else
    wk = 1 : ceil(wkmax / cellSize);
end
%第二个地方就是：本方案它的窗口是以1进行逐级递增的，我们是否可以进行改进？？？


% wk = wkmax;
%% Define elevation thresholds based on supplied slope tolerance 根据提供的坡度容许偏差定义高程阈值

eThresh = slopeThreshold * (wk * cellSize);%这个地方，是不是由于坡度阈值的固定，进而给高差阈值造成影响
%这里是我们需要设法进行改进的地方：开运算中的阈值
%换句话说，我们最应该改进的是，slopeThreshold 如何让它在不同地形处，能够有所适应性地变化
%此外，看了一下，此处的高程阈值分别为0.2 0.4 0.6 0.8 1.0 1.2 1.4 ... 3.2 非常不合理
%此处求 et并没有用到es

%% Perform iterative filtering.迭代滤波

isObjectCell = logical(zeros(size(lastSurface)));%logical 将double类型的矩阵转换为逻辑值

% for i = 1:length(wk)
%     thisSurface = imopen(lastSurface,strel(strelShape,wk(i)));%%%记录 imopen之后的结果是什么？？？什么类型的矩阵？
% %     isObjectCell1=isObjectCell;
%     thisSurface1=thisSurface;
%     thisSurface1(thisSurface1<320)=randperm(1,330:340);
%     isObjectCell = isObjectCell | (lastSurface - thisSurface > eThresh(i));
%     lastSurface = thisSurface;
%     surf(thisSurface1,'edgecolor','none')
%     figure(i)
%     title(['这是第',i,'次迭代'])
% %     if i>1
% %     if sum(sum(isObjectCell1))/sum(sum(isObjectCell))<=1.000004
% %         thisSurface = imopen(lastSurface,strel(strelShape,wk(i+1)));
% %         isObjectCell = isObjectCell | (lastSurface - thisSurface > eThresh(i+1));
% %         break;
% %     end
% %     end
% %     thisSurface=lastSurface;
% end
fprintf("网格有%d行\n",size(lastSurface,2))
        fprintf("网格有%d列\n",size(lastSurface,1))
        hang=input("小格网行数");
        lie=input("小格网列数");
for i = 1:length(wk)
    thisSurface = imopen(lastSurface,strel(strelShape,wk(i)));
    if varargin{7}==2
         result=huafen(thisSurface,hang,lie);
         gaocha=calpodu(result);
         podu=gaocha/density;
         eThresh(i)= podu*(wk(i) * cellSize)/100;
    end
    isObjectCell = isObjectCell | (lastSurface - thisSurface > eThresh(i));
    lastSurface = thisSurface;
end

