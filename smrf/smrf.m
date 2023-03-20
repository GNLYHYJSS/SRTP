% smrf
% A Simple Morphological Filter for Ground Identification of LIDAR point clouds.
%   简易形态学滤波 为点云数据的地面分类
%
% Syntax 语法
%   [ZIfin R isObject ZIpro ZImin isObjectCell] = smrf(x,y,z,'c',c,'s',s,'w',w);
%
%
% Description
%   SMRF is designed to apply a series of opening operations against a
%   digital surface model derived from a LIDAR point cloud, with the dual
%   purpose of creating a gridded model of the ground surface (ZIfin) (and
%   its referencing matrix R) and a vector of boolean values for each tuple
%   (x,y,z) describing it as either ground (0) or object (1).
%   SMRF 对数字表面模型DSM（从点云数据获得的）进行一系列的开运算，
%   它同时有两个目的：
%   目的1：创建一个地面的格网模型ZIfin（以及它的参考矩阵R）
%   目的2：得到一个bool类型的变量
%   描述每一个点云tuple是地面点还是地物点（0是地面点，1是地物点）

%   SMRF must be minimally called with x,y,z (all vectors of the same
%   length) as well as a cellsize (c), a slope threshold value (s), and a
%   maximum window size (w).  The slope threshold value governs the
%   identification process, and roughly corresponds to the maximum slope of
%   the terrain you are working with.  The maximum window size defines a
%   window radius (in map units), and corresponds to the size of largest 
%   feature to be removed.
%   SMRF需要x,y,z还有格网尺寸c,坡度阈值s，最大化窗口尺寸w.   
%   坡度阈值s控制着分类的进程，并且大致和地面的最大化坡度一致
%   最大化窗口尺寸w确定一个窗口半径，和要去除的最大地物大小一致   
%   
%
% Requirements
%   SMRF requires John D'Errico's consolidator.m and inpaint_nans.m files.
%   These are available via the Mathworks File Exchange Program at:
%   http://www.mathworks.com/matlabcentral/fileexchange/8354
%   http://mathworks.com/matlabcentral/fileexchange/4551
%   需要两个别人的函数consolidator.m和inpaint_nans.m   

%   SMRF also requires two subfunctions, createDSM.m and
%   progressiveFilter.m.  These were written as subfunctions for
%   pedagogical purposes.  If the optional 'net cutting' feature is used to
%   remove large buildings with smaller windows, createNet.m must be
%   accessible as well.
%   还需要两个子函数：createDSM.m和progressiveFilter.m    pedagogical purposes教学目的
%   还需要createNet.m，当可选的"网络切割"功能用于去除大型建筑（通过较小的窗口实现）；具体意思还不是太了解

%   Finally, SMRF also requires that the Mathworks Mapping Toolbox is 
%   installed.
%   安装Mapping Toolbox工具
%
% Input Parameters 输入变量
%   x,y,z     - Equally sized vectors defining the points in the cloud
%   确定点在点云中的位置坐标
%
%   'c',c     - Cell size, in map units, for the final grid.  The cell size
%               should generally be close to the mean x,y density of your
%               point cloud.
%    格网尺寸c（地图坐标尺寸下的）：大体上接近点云的平均x,y密度
%   'xi',xi   - Alternatively, the user can supply a vector of values to 
%   'yi',yi     define the grid
%    格网的位置？
%   's',s     - Defines the maximum expected slope of the ground surface
%               Values are given in dz/dx, so most slope values will be in
%               the range of .05 to .30.
%    坡度阈值s：地面的最大预期坡度 值等于dz/dx，大部分的坡度值在0.05到0.3
%   'w',w     - Defines the filter's maximum window radius.  w最大窗口大小
%   'w',[0 w]   Alternatively, the user can supply his or her own vector of
%   'w',[1:5:w] window sizes to control the open process.  
%   Alternatively：同样地，用户也可以自己设置窗口半径的大小变化过程
%   'et',et   - Defines the elevation threshold that expresses the maximum
%               vertical distance that a point may be above the prospective
%               ground surface created after the opening operation is
%               completed.  
%               These values are typically in the range of 0.25to 1.0 meter.  
%               An elevation threshold must be supplied in order for SMRF to return an isObject vector.
%   et:高程阈值，在开运算结束之后会得到一个预期的地表模型，高程阈值就是点云可能超过该地表模型的最大高度
%               et的值，通常在0.25到1米之间；et必须有序提供，才能实现滤波
%
%   express：表示、表露
%   'es',es   - Elevation scaling factor that scales the elevation
%               threshold (et) depending on the slope of the prospective
%               digital surface model (ZIpro) created after the smrf filter
%               has identified all nonground points in the minimum surface.
%               
% Elevation scaling factors generally range from 0.0 to 2.5,
% with 1.25 a good starting value.  
% If no es parameter is supplied, the value of es is set to zero.
%   es：高程比例因子，它缩放高程阈值（依据的是：smrf在最小的地面已经辨别出了所有的非地面点之后的DSM的坡度信息）
%   ZIpro:DSM   高程比例因子，通常在0.0到2.5，以1.25作为初始值，如果没有提供的话，默认值为0
%   
%   'inpaintMethod',ipm
%               If this parameter is supplied, it controls the argument参数 
%               passed to D'Errico's inpaint_nans method.  The default
%               value is 4.
%   ipm默认值是4，该值会被传参到 那个人的inpaint_nans method
%   'cutNet',netSize
%               Cuts a net of spacing netSize (map coordinates) into ZImin
%               before further processing.  
% This can help to remove large buildings without the need for extremely large filter windows. 
% Generally, netSize should be set to the largest window radius used (w).
%   netSize:通常被设为最大窗口半径w，
%   这有助于移除大型建筑物，而不需要超大的过滤器窗口
% Output Parameters 输出参数
%
%   ZIfin       A digital surface model (DSM) of the ground.  If an
%               elevation threshold is not provided, the final DSM is set 
%               equal to the prospective DSM (see below).
%
%   R           A referencing matrix that relates the image file (ZIfin) to
%               map coordinates.  See worldfileread for more information.
%   R将图像文件（ZIfin）与地图坐标关联的参考矩阵。有关详细信息，请参阅worldfileread
%   isObject    A logical vector, equal in length to (x,y,z), that
%               describes whether each tuple is ground (0) or nonground (1)
%
%   ZIpro       The prospective ground surface created after the smrf
%               algorithm has identified nonground cells in the initial
%               minimum surface (ZImin).  在SMRF分类出地物点格网之后创建的一个预期地面
% It is created by inpainting all  empty, outlier, or nonground cells from the minimum surface.
%
%   ZImin       The initial minimum surface after smrf internally calls
%               createDSM.m.
%internally adv.内部地
%   ZImin:最初的DSM（单纯格网化之后，取最小值生成的表面）
%   isObjectCell 
%               Cells in ZImin that were classified as empty,outliers,or
%               objects during SMRF's run.
%       某些格网（在SMRF运行期间被分类为空值、异常值或者是地物的格网）


% Examples
% 
% % Test SMRF against ISPRS data set.
% 
% % Download reference LIDAR data
% url = 'http://www.itc.nl/isprswgIII-3/filtertest/Reference.zip';
% fn = 'Reference.zip';
% urlwrite(url,[tempdir,'\',fn]);
% unzip([tempdir,'\',fn], tempdir);
% 
% % Read data
% M = dlmread([tempdir,'\samp11.txt']);
% x = M(:,1);
% y = M(:,2);
% z = M(:,3);
% gobs = M(:,4);  % 0 is Ground, 1 is Object
% clear M;
% 
% % Declare parameters for this sample (Pingel et al., 2011)
% c = 1;
% s = .2;
% w = 16;
% et = .45;
% es = 1.2;
% 
% % Run filter
% [ZI R gest] = smrf(x,y,z,'c',c,'s',s,'w',w,'et',et,'es',es);
% 
% % Report results
% ct = crosstab(gobs,gest)
% 
% % View surface
% figure;
% surf(ZI,'edgecolor','none'); axis equal vis3d
%绘制结果的DEM，带有颜色渲染
% References:   The filter was succesfully tested against the Sithole and
%               Vosselman's (2003) ISPRS LIDAR Dataset
%               (http://www.itc.nl/isprswgIII-3/filtertest/).
%
%
%%
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
%
%
% See Also:
% worldfilewrite.m, 



function [ ZIfin R isObject ZIpro ZImin isObjectCell] = smrf(x,y,z,varargin)%varargin是可以输入多个变量的地方
% R 实现像素坐标与格网坐标的转化，ZIPro得到的是临时的DEM ，isobjectcell代表格网是否是地物格网
if nargin < 9 %nargin计算函数输入变量数
    error('Not enough arguments.  Minimum call: smrf(x,y,z,''c'',c,''s'',s,''w'',w)');
end
 
dependencyCheck = [exist('consolidator') exist('inpaint_nans')];%dependencyCheck 是检查依赖项是否安装
if any(dependencyCheck==0)%返回值为0，则证明没有安装
    disp('The smrf algorithm requires that consolidator.m and inpaint_nans.m are accessible.');
    disp('Both of these functions were written by John D''Errico and are available through the');
    disp('Mathworks file exchange.');
    disp('consolidator is located at http://www.mathworks.com/matlabcentral/fileexchange/8354');
    disp('inpaint_nans is located at http://mathworks.com/matlabcentral/fileexchange/4551');
    error('Please acquire these files before attempting to run smrf again.');
end
% Initialize possible input values  初始化输入的值
cellSize = [];%格网尺寸
slopeThreshold = [];%坡度阈值
wkmax = [];%最大化窗口尺寸,滤波元素（结构元素）最大尺寸
xi = [];
yi = [];
elevationThreshold = [];%高程阈值
elevationScaler = [];%高程缩放因子

% Initialize output values 初始化输出的值
ZIfin = [];%最终的地面
R = [];%将图像文件（ZIfin）与地图坐标关联的旋转矩阵。
isObject = [];%是否为地物

% Declare other global variables 定义一些全局变量
inpaintMethod = 4;  % Springs  inpaint油漆、去瑕疵
cutNetSize = [];%剪切网络大小
isNetCell = [];%是网络单元格？


%% Process extra arguments  处理其他的变量
%[ZIfin R isObject ZIpro ZImin isObjectCell] = smrf(x,y,z,'c',c,'s',s,'w',w);
% [ZI R gest] = smrf(x,y,z,'c',c,'s',s,'w',w,'et',et,'es',es);
%调用处的语句如上
i = 1;
while i<=length(varargin)    
    if isstr(varargin{i}) %如果是str类型，就用switch进行判断，其实有点类似于读取文件
        switchstr = lower(varargin{i});%lower将字符串转换为小写char类型 
        switch switchstr  
            case 'c' % Cell size (required, or xi and yi must be supplied) 格网大小c
                cellSize = varargin{i+1};
                i = i + 2;
            case 's' % Slope tolerance (required)  坡度阈值s
                slopeThreshold = varargin{i+1};
                i = i + 2;
            case 'w' % Maximum window size, in map units (required)  最大化窗口尺寸w
                wkmax = varargin{i+1};
                i = i + 2;  
            case 'et'   % Elevation Threshold (optional)  高程阈值et
                elevationThreshold = varargin{i+1};
                i = i + 2;
            case 'es'   % Elevation Scaling Factor (optional)  高程缩放因子
                elevationScaler = varargin{i+1};
                i = i + 2;
            case 'xi'   % A supplied vector for x  
                xi = varargin{i+1};
                i = i + 2;
            case 'yi'   % A supplied vector for y
                yi = varargin{i+1};
                i = i + 2;        
            case 'inpaintmethod'  % Argument to pass to inpaint_nans.m 传递到函数inpaint_nans.m的一个变量
                inpaintMethod = varargin{i+1};
                i = i + 2;
            case 'cutnet'   % Support to a cut a grid into large datasets cutnet 支持将网格剪切为大型数据集剪切网
                cutNetSize = varargin{i+1};
                i = i + 2;
            case 'objectMask'%地物掩膜
                objectMask = varargin{i+1};
                i = i + 2;
            otherwise
                i = i + 1;
        end
    else
        i = i + 1;
    end
end    


%% Check for a few error conditions  检查有无错误的条件

if isempty(slopeThreshold)
    error('Slope threshold must be supplied.');%必须提供坡度阈值s
end

if isempty(wkmax)
    error('Maximum window size must be supplied.');%必须提供最大化窗口尺寸
end

if isempty(cellSize) && isempty(xi) && isempty(yi)
    error('Cell size or (xi AND yi) must be supplied.');%要么有窗口尺寸c,要么有（xi和yi）
end

if isempty(xi) && ~isempty(yi)
    error('If yi is defined, xi must also be defined.');%如果有了yi,必须要有xi
end

if ~isempty(xi) && isempty(yi)
    error('If xi is defined, yi must also be defined.');%如果有了xi,必须要有yi
end

if ~isempty(xi) && ~isvector(xi)  
    error('xi must be a vector');%xi必须为一个向量
end

if ~isempty(yi) && ~isvector(yi)
    error('yi must be a vector');%yi必须为一个向量
end

if ~isempty(xi) && (abs(xi(2) - xi(1)) ~= abs(yi(2) - yi(1)))
    error('xi and yi must be incremented identically');%xi和yi的增量必须相同
end

if isempty(cellSize) && ~isempty(xi) && ~isempty(yi)
    cellSize = abs(xi(2) - xi(1));%如果，没有提供格网尺寸，但是有xi和yi，则格网尺寸c=xi(2)-xi(1)
end

if ~isempty(elevationThreshold) && isempty(elevationScaler)%如果，高程阈值不为空，高程缩放因子为空的话，将高程缩放因子设为默认值0
    elevationScaler = 0;
end

%% Create Digital Surface Model  创建DSM  有xi,yi则用xi,yi;没有则用cellSize作参数
if isempty(xi)
    [ZImin R isEmptyCell xi yi] = createDSM(x,y,z,'c',cellSize,'type','min','inpaintMethod',inpaintMethod);
else
    [ZImin R isEmptyCell] = createDSM(x,y,z,'xi',xi,'yi',yi,'type','min','inpaintMethod',inpaintMethod);
end

%% Detect outliers   剔除噪点,此处的坡度阈值不是0.2，而是采用了5，目的是为了去除粗差点
%输入的是 DSM\格网尺寸c、坡度阈值、最大窗口大小（此处也是只设置成了1，即没有迭代的过程）
[isLowOutlierCell] = progressiveFilter(-ZImin,'c',cellSize,'s',5,'w',1,1); 
%最后的低粗差点被赋值为1
%此外，此处暂时没有发现判断出低粗差点的作用，至少它应该在探测地物中的开运算之前有所展示作用啊！！！
% ZImin(isLowOutlierCell)=334;

%这里不知道作用是什么   本实例也没有给出相关参数，故不会执行只一步，但是很有价值和意义
%% Cut a mesh into Zmin, if desired  如果需要，将网格切割为Zmin，ZImin为初始表面，ZIpro临时DEM
if ~isempty(cutNetSize)%如果给出了这一尺寸，就进行下面的操作
    [ZInet isNetCell] = createNet(ZImin,cellSize,cutNetSize);%WK,WKmax滤波窗口半径（结构元素半径）
else
    ZInet = ZImin;%如果没有给出要去除的最大建筑物的尺寸的话，就按照原来的ZImin进行下一步操作
    isNetCell = logical(zeros(size(ZImin))); %logical将数值转换为逻辑值
end

%% Detect objects 探测地物

[isObjectCell] = progressiveFilter(ZInet,'c',cellSize,'s',slopeThreshold,'w',wkmax,2); 

%% Construct a prospective ground surface 创建一个期望的地面

ZIpro = ZImin;%ZImin为初始表面，ZIpro临时DEM
%将低粗差点、地物、空格网、被去除的最大建筑物格网的值赋值为 不定值NAN
ZIpro(isEmptyCell | isLowOutlierCell | isObjectCell | isNetCell) = nan; %此处的ZIpro不能绘制，因为它的值不全
ZIpro = inpaint_nans(ZIpro,inpaintMethod);%将剩余之后的矩阵值进行内插，不需要取最小值，单纯内插就行
isObjectCell = isEmptyCell | isLowOutlierCell | isObjectCell | isNetCell;%最终的地物矩阵也是几类情况的综合


%% Identify ground ...
% based on elevationThreshold and elevationScaler, if provided

if ~isempty(elevationThreshold) && ~isempty(elevationScaler)

    % Identify Objects
        % Calculate slope
    [gx gy] = gradient(ZIpro / cellSize); %数值梯度 gradient 求矩阵的梯度
    gsurfs = sqrt(gx.^2 + gy.^2); % Slope of final estimated ground surface 最终地面的坡度
    clear gx gy  
            
    %% 梯度的求法
    % Get Zpro height and slope at each x,y point
    iType = 'spline'; %spline 样条曲线
    [r c] = map2pix(R,x,y); %map2pix将所有点云的地图坐标转换为像素坐标
    ez = interp2(ZIpro,c,r,iType);%ez在DEM上采用插值方法生成原始点云对应的地面高程ez（列向量），
    % 根据原始点云的高程z与插值高程ez生成高程差值，将高程差值的绝对值 与高差阈值et比较，
    % 若 <et，则判为地面点，否则判为非地面点

    %在生成的梯度上，进行插值得到原始点云对应的梯度？？？
    SI = interp2(gsurfs,c,r,iType);  %interp2：meshgrid格式的二维网格数据的插值
    clear r c
    %elevationThreshold 在此处发挥的作用；这个公式？？？
    requiredValue = elevationThreshold + (elevationScaler * SI);%requiredValue指的是高程阈值
    isObject = abs(ez-z) > requiredValue;   
    clear ez SI requiredValue 
    
    
    % Interpolate final ZI  %内插最终DEM
   F = TriScatteredInterp(x(~isObject),y(~isObject),z(~isObject),'natural');%TriScatteredInterp对散点数据插值
   [XI,YI] = meshgrid(xi,yi); %meshgrid二维和三维格网
   ZIfin = F(XI,YI);
else
   warning('Since elevation threshold and elevation scaling factor were not provided for ground identification, ZIfin is equal to ZIpro.');
   ZIfin = ZIpro;%最终DEM 等于 临时的较为理想的DEM
end
  


end  % Function end