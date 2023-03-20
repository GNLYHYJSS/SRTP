function podu=calpodu(shuju)
[~,~,band]=size(shuju);
mins=zeros(band,1);
maxs=zeros(band,1);
for i=1:band
    mins(i)=min(min(shuju(:,:,i)));
    maxs(i)=max(max(shuju(:,:,i)));
end
podu=0.85*max(maxs-mins);
end