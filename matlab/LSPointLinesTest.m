function LSPointLinesTest(n)
%Creates n random lines in 2D space less than a distance of 1 from the origin
%and finds point minimizing sum of squared distances.

nhat = unifrnd(-1,1,2,n);
nhat = nhat./repmat(sqrt(sum(nhat(1:2,:).^2)),2,1); %make unit length
d = unifrnd(-1,1,1,n); %constrain distance from origin to be less than 1
L = [nhat;d];

%scale normalized lines back to homogeneous lines with random scale
w = repmat(unifrnd(-10,10,1,n),3,1); 

%Solve for least squares point x
x = LSPointLines(w.*L)

%Display results on [-1 1] x [-1 1]
figure;
for i = 1:n
    nhati = nhat(:,i);
    di = d(i);
    x1 = (-di+nhati(2))/nhati(1);
    x2 = (-di-nhati(2))/nhati(1);
    y1 = (-di+nhati(1))/nhati(2);
    y2 = (-di-nhati(1))/nhati(2);
    
    xep = [];yep=[];
    if abs(x1)<1
        xep = [xep;x1];
        yep = [yep;-1];
    end
    if abs(x2)<1
        xep = [xep;x2];
        yep = [yep;1];
    end
    if abs(y1)<1
        xep = [xep;-1];
        yep = [yep;y1];
    end
    if abs(y2)<1
        xep = [xep;1];
        yep = [yep;y2];
    end
    
    line(xep,yep);
    hold on;
end

plot(x(1),x(2),'.','MarkerSize',18);
set(gca,'XAxisLocation','origin');
set(gca,'YAxisLocation','origin');
xlabel('x');
ylabel('y');
set(gca,'FontSize',12);