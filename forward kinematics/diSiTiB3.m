function y = diSiTiB3()
    A = [-1 0 0; 0 -1 0; 0 0 1];
    det1 = det(A)
    inverseA = inv(A);
    transposeA = A.';
    if (inverseA == transposeA) %check if inverse and transpose are equal
        y = 'equal';
    else
        y = 'not equal';
    end
end

