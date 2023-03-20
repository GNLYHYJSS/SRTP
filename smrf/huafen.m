function result=huafen(shuju,m,n) % m:行划分，n:列划分
number=m*n;
[row,col]=size(shuju);
p=row/m;
q=col/n;
result=zeros(p,q,number);
rows=1;
cols=1;
for i=1:number
    result(:,:,i)=shuju(p*(rows-1)+1:p*rows,q*(cols-1)+1:q*cols);
end
end