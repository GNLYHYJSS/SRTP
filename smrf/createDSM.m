% createDSM
% Simple utility to construct a Digital Surface Model from LIDAR data.
% utility 公用事业;实用;效用;有用
% 从激光雷达数据 构建 数字表面模型DSM 的简单实用程序。
% Syntax 语法
%   [DSM R isEmptyCell xi yi] = createDSM(x,y,z,varargin);
%
%
% Description 简述
%   createDSM takes as input a three dimensional point cloud (usually LIDAR
%   data) and creates an initial ground surface, useful for further
%   processing routes that can extract ground and identify objects in
%   scene.  Required input (in addition to the point cloud) is a cellSize
%   (in map coordinates).  The user may, instead, optionally specify xi and
%   yi, two vectors to which the data will are then snapped.  By default
%   createDSM creates a minimum surface, in which duplicate cell values are
%   reduced to their minimum.  However, the user may specify other values
%   (e.g., 'mean', 'max') if desired.
%   extract：提取   in scene 在现场
%   createDSM输入的是：一个三维的点云点的坐标；输出的是：最初的地面ZImin，用来为进一步的提取地面和分类地物有用处
%   输入要素还需要：cellSize格网尺寸（在地图坐标下的）或者要xi和yi
%   createDSM将三维点云（通常为激光雷达数据）作为输入，并创建初始地面曲面，这对于进一步处理路线非常有用，
%   可以提取地面并识别场景中的对象。所需输入（除了点云）是cellSize（在地图坐标中）。
%   相反，用户可以 选择性地 指定xi和yi，这两个向量随后将被捕捉到数据。
%   默认情况下，createDSM会创建一个最小曲面，其中每个格网中选取的是 最小值。
%   但是，如果需要，用户可以指定其他值（例如，"平均值"、"最大值"）。
%
%   specify v.具体说明、详列  snap  v.猛地咬住，断裂崩裂
% Requirements
%   Requires John D'Errico's consolidator.m and inpaint_nans.m files.
%   These are available via the Mathworks File Exchange Program at:
%   http://www.mathworks.com/matlabcentral/fileexchange/8354
%   http://mathworks.com/matlabcentral/fileexchange/4551
%
%   Also requires that the Mathworks Mapping Toolbox is installed.
%
%
% Input Parameters
%   x,y,z     - Equally sized vectors defining the points in the cloud点云坐标
%
%   'c',c     - Cell size, in map units, for the final grid.  The cell size
%               should generally be close to the mean x,y density of your
%               point cloud.格网尺寸，地图单元中，为最终的格网；格网尺寸普遍大体上应当
%               接近你的点云的平均x和y密度
%   'xi',xi   - Alternatively, the user can supply a vector of values to 
%   'yi',yi     define the grid
%               同样地，用户可以使用一个向量值xi,yi定义格网
%   'type',t    String value or function handle specifying the basis for
%               consolidation.  Possible values include 'min', 'median',
%               'mean', 'max'.  See consolidator.m for all possible inputs.
%   specify v.指定，详列，具体说明  the basis for consolidation 合并的依据
%               consolidation.m函数所需要的参数，求格网内的 最小值、最大值、均值、中值
%   'inpaintMethod',ipm
%               If this parameter is supplied, it controls the argument
%               passed to D'Errico's inpaint_nans method.  The default
%               value is 4.
%               默认值是4
%
% Output Parameters
%
%   DSM         A digital surface model (DSM) of the ground. 
%
%   R           A referencing matrix that relates the image file (ZIfin) to
%               map coordinates.  See worldfileread for more information.
%               旋转矩阵，实现image file和map coordinate的转换
%               图像文件（ZIfin）与地图坐标关联的参考矩阵。有关详细信息，请参阅worldfileread
%   isEmptyCell An image mask describing whether the DSM was empty or not
%               at that location.  
%   描述DSM在该位置是否为空的图像掩码  空的话，该处的值是？非空的话，该处的值是？
%   xi,yi       Vectors describing the range of the image.
%
%
% Examples: 使用例子
% % Download reference LIDAR data
% url = 'http://www.itc.nl/isprswgIII-3/filtertest/Reference.zip';
% fn = 'Reference.zip';
% urlwrite(url,[tempdir,'\',fn]);
% unzip([tempdir,'\',fn], tempdir);
% 
% % Read data
% M = dlmread([tempdir,'\samp11.txt']); dlmread 将ASCII码分割的数值数据文件读到矩阵中
% x = M(:,1);
% y = M(:,2);
% z = M(:,3);
% gobs = M(:,4);  % 0 is Ground, 1 is Object  gobs记录是地面点还是地物点
% clear M;
% 
% % Create a minimum surface 
% [ZImin R isEmptyCell] = createDSM(x,y,z,'c',1,'min');
% 
% % Write data to geotiff 将数据写到samp11.tif中去
% imwrite(ZImin,'samp11.tif','tif');
% worldfilewrite(R,'samp11.tfw');
%
%
%
%  
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met  如果满足以下条件，则允许以源代码和二进制形式重新分发和使用，无论是否进行修改：
% 
%     * Redistributions of source code must retain the above copyright 
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright 
%       notice, this list of conditions and the following disclaimer in 
%       the documentation and/or other materials provided with the distribution
%       retain v.保留  disclaimer n.免责声明 
%重新分发源代码必须保留上述版权声明、此条件列表和以下免责声明。
%二进制形式的再版必须在随发行版提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明

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
% See Also:
% worldfilewrite.m, 

function [DSM,R,isEmptyCell xi yi] = createDSM(x,y,z,varargin) 

% Define inputs 初始化输入的值

cellSize = [];
inpaintMethod = [];
xi = [];
yi = [];
cType = [];

% Define outputs  初始化输出的值

DSM = [];
R = [];
isEmptyCell = [];

%% Process supplied arguments

i = 1;
while i<=length(varargin)    
    if isstr(varargin{i})   
        switchstr = lower(varargin{i});
        switch switchstr
            case 'c'
                cellSize = varargin{i+1};
                i = i + 2;
            case 'inpaintmethod'
                inpaintMethod = varargin{i+1};
                i = i + 2;
            case 'xi'
                xi = varargin{i+1};
                i = i + 2;
            case 'yi'
                yi = varargin{i+1};
                i = i + 2;
            case 'type'
                cType = varargin{i+1};
                i = i + 2;
            otherwise
                i = i + 1;
        end
    else
        i = i + 1;
    end
end    


    if isempty(cType)  %如果cType为空值，则赋予它'min'
        cType = 'min';
    end
    if isempty(inpaintMethod) %如果inpaintMethod为空值，则赋予它4，表示内插方法为Springs 
        inpaintMethod = 4; % Springs as default method
    end
% relief v.凸显立体效果
    % define cellsize from xi and yi if they were not defined
    if ~isempty(xi) & ~isempty(yi)  %如果xi,yi都不为空，则cellSize格网尺寸为xi(2)-xi(1)
        cellSize = abs(xi(2) - xi(1));
    end
    
    if isempty(cellSize) %如果cellize格网尺寸为空，则报错，提示：必须存在
        error('Cell size must be declared.');
    end
    
    % Define xi and yi if they were not supplied   ceiling 天花板 floor 地板
    if isempty(xi) & isempty(yi)%如果，xi和yi为空值的话，
        xi = ceil2(min(x),cellSize):cellSize:floor2(max(x),cellSize); 
        yi = floor2(max(y),cellSize):-cellSize:ceil2(min(y),cellSize); 
    end

    % Define meshgrids格网 and referencing matrix
    [XI YI] = meshgrid(xi,yi); 
    R = makerefmat(xi(1),yi(1),xi(2) - xi(1),yi(2) - yi(1)); %makerefmat制作仿射空间矩阵

    % Create gridded values
    xs = round3(x,xi); %将x变为xi
    ys = round3(y,yi);

    % Translate (xr,yr) to pixels像素坐标 (r,c)
    [r c] = map2pix(R,xs,ys);  %map2pix将地图坐标转换成pixel坐标，（xs,ys）地图坐标；(r,c)pixel坐标
    r = round(r);  % Fix any numerical irregularities 四舍五入
    c = round(c);  % Fix any numerical irregularities
    %  Translate pixels to single vector of indexed values 
    idx = sub2ind([length(yi) length(xi)],r,c);  %将下标转换成线性索引
    
    % Consolidate those values according to minimum 根据最小值合并这些值
    %Consolidate v.巩固,合并，结成一体  
    [xcon Z] = consolidator(idx,z,cType);%取z的最小值

    % Remove any NaN entries.  How these get there, I'm not sure. NaN：不定值  如0/0
    %inf：无穷大 如1/0
    Z(isnan(xcon)) = [];  
    xcon(isnan(xcon)) = [];
    
    % Construct image 
    DSM = nan(length(yi),length(xi));
    DSM(xcon) = Z; 
    
    isEmptyCell = logical(isnan(DSM)); %logical将数值转换成逻辑值
    
    % Inpaint NaNs 
    if (inpaintMethod~=-1 & any(isnan(DSM(:))))
        DSM = inpaint_nans(DSM,inpaintMethod);
    end
    
%     tStop = tStart - tStop;

end




function xr2 = floor2(x,xint) %自定义的函数floor2  floor地板
    xr2 = floor(x/xint)*xint; %floor 向负无穷舍入
end

function xr2 = ceil2(x,xint) %自定义的函数ceil2  ceiling天花板
    xr2 = ceil(x/xint)*xint; %ceil  向正无穷舍入
end

function xs = round3(x,xi) % Snap vector of x to vector xi 将向量x折断成xi
    dx = abs(xi(2) - xi(1));
    minxi = min(xi);
    maxxi = max(xi); 

    %% Perform rounding on interval dx 对间隔dx执行四舍五入 round函数四舍五入为最小的小数或整数
    xs = (dx * round((x - minxi)/dx)) + minxi;

    %% Outside the range of xi is marked NaN  xi范围之外被标记为NaN（不确定值）
    % Fix edge cases   NaN：不定值  如0/0  inf：无穷大 如1/0

    xs((xs==minxi-dx) & (x > (minxi - (dx)))) = minxi;
    xs((xs==maxxi+dx) & (x < (maxxi + (dx)))) = maxxi;
    % Make NaNs
    xs(xs < minxi) = NaN;
    xs(xs > maxxi) = NaN;
end