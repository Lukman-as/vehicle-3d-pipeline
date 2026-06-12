function vphorizon=findVP(horizonLine,sample2D)

lineSamples=[];
for i=1:2:size(sample2D,1)
    lineSamples=[lineSamples;sample2D(i,:),sample2D(i,:)-sample2D(i+1,:)];
end

options = optimset( 'Display','off','TolFun',1e-11,'TolX', 1e-11);
orig=horizonLine(:,1)';
dir=(horizonLine(:,1)-horizonLine(:,2))';
dir=dir/norm(dir);

[newscale, resnorm, residual, exitflag] = fminunc('minimumdistance2point',1,options, orig,dir,lineSamples);


vphorizon=orig+newscale*dir;

end