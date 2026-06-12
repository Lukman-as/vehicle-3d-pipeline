function raw_data = removeOutlier(raw_data)
data = raw_data(2,:);
%Before filtering out
[praw,edges] = histcounts(data,'Normalization','probability');
sprintf('Before filtering: The range is from %.2f to %.2f',min(data),max(data))

kmax = 4; %Normal distribution kurtosis
alpha1 = 0.0001;
alpha2 = 0.0001;
k = kurtosis(data)
while k > kmax
%     size(raw_data, 2)
%     size(data, 2)
    assert(size(raw_data, 2) == size(data, 2)); %Matching removed stuff
    [data TFrm,~] = rmoutliers(data,'percentiles',100*[alpha1,1-alpha2]);
    raw_data = raw_data(:,TFrm == 0);
    k = kurtosis(data);
end
k = kurtosis(data)
sprintf('The range is from %.2f to %.2f',min(data),max(data))
if true
    pfiltered = histcounts(data,edges,'Normalization','probability');
    figure;
    edges = (edges(1:end-1)+edges(2:end))/2; %Normalizing the edges
    plot(edges,praw,'LineWidth',2);
    hold on;
    plot(edges,pfiltered,'r','LineWidth',2);
    set(gca,'XLim',[6 9]);
end
