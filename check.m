% this function is used to verify the accuracy of Inverse Kinematics results
thetas = [0,0,0,0,0];
diffs = []; 
diff = 0;
for i = pi/30:pi/30:pi/6
    for j = pi/30:pi/30:pi/6
        for k = pi/30:pi/30:pi/6
            for s = pi/30:pi/30:pi/6
                for t = pi/30:pi/30:pi/6
                    thetas = [i,j,k,s,t];
                    [Tend, left] = fLeftHandH25(thetas);
                    results = LeftArmSolutionRuixin(left);
                    for g = 1:1:5
                        diff = diff + (thetas(g) - results(g))^2;                       
                    end
                    diffs = [diffs;diff];
                    diff = 0;
                end
            end
        end
    end
end