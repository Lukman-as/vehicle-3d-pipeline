function newMatrix=cameraOptimization(oldMatrix,sample2D,center,K)

arrayMatrix=oldMatrix(:);

options = optimset( 'Display','off','TolFun',1e-11,'TolX', 1e-11);

[outputMatrix, resnorm, residual, exitflag] = fminunc('costFunction',arrayMatrix,options, sample2D, center,K);

newMatrix=reshape(outputMatrix(:),3,3);
end