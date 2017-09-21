function y = diSiTiB1()
    A = [1/sqrt(3) -sqrt(3)/2 sqrt(3)/6;1/sqrt(6) sqrt(3)/3 -2/sqrt(6);...
        0 1/sqrt(3) -sqrt(2)/2];
    det1 = det(A)
    inverseA = inv(A);
    transposeA = A.';
    if (inverseA == transposeA) %check if inverse and transpose are equal
        y = 'equal';
    else
        y = 'not equal';
    end
end

