function y = diSanTiB2()
    A = [-5 4 -1; -4 1 -3; 11 1 10];
    B = [-30; -13; 21];
    solution = linsolve(A,B) %solution of the linear equations
    conditionNum = cond(A) %condition number of the system
    errSolution = zeros(10,3);
    j = 1;
    for i=0.001:0.001:0.01
        B(1,1) = B(1,1) + i; %add noise to the equations
        errSolution(j,:) = linsolve(A,B);
        j=j+1;
    end
    j=1:10
    figure %plot the error
    subplot(2,2,1)
    plot(j,solution(1)-errSolution(j,1));
    title('x');
    subplot(2,2,2)
    plot(j,solution(2)-errSolution(j,2));
    title('y');
    subplot(2,2,3)
    plot(j,solution(3)-errSolution(j,3));
    title('z');
end