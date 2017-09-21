function y = diSiTiB2()
    A = [sqrt(6)/4 -1/2 -sqrt(6)/4;sqrt(2)/4 sqrt(3)/2 -2/4;...
        sqrt(2)/2 0 sqrt(2)/2];
    det1 = det(A)
    inverseA = inv(A);
    transposeA = A.';
    if (inverseA == transposeA) %check if inverse and transpose are equal
        y = 'equal';
    else
        y = 'not equal';
    end
end

