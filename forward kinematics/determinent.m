function D = determinent(A) %Time complexity is n^3
    if size(A,1)~=size(A,2)
        message='matrix not square; no proper determinant'
    else
        if max(size(A))==2
            D = (A(1,1)*A(2,2))-(A(1,2)*A(2,1)); %basic formula of determinent
        else
            for i=1:size(A,1)
                D_temp=A;
                D_temp(1,:)=[ ]; %pre-allocate space
                D_temp(:,i)=[ ];
                if i==1
                    D=(A(1,i)*((-1)^(i+1))*determinent(D_temp)); %recursive
                else
                    D=D+(A(1,i)*((-1)^(i+1))*determinent(D_temp));
                end
            end
        end
    end
end