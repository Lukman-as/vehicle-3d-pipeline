function x=LSPointLines(L)
%Find point minimizing sum of squared distances to lines in L.

n = size(L,2);
nhat = L(1:2,:);
nhatmag = sqrt(sum(nhat(1:2,:).^2));
nhat = nhat./repmat(nhatmag,2,1); %make unit length
d = L(3,:)./nhatmag; 

%Solve for least squares point x
Nsq = zeros(2);
for i = 1:n
    nhati = nhat(:,i);
    Nsq = Nsq + nhati*nhati';
end

x = Nsq\sum(-nhat.*repmat(d,2,1),2);