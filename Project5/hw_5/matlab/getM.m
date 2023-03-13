function M = getM(n_seg, n_order, ts)
    M = [];
    n_coef=n_order+1;
    for k = 1:n_seg
        M_k = zeros(n_coef,n_coef);
        for j=1:n_coef/2
            for i=j-1:n_order
                M_k(j,i+1)=factorial(i)/factorial(i-(j-1))*0^(i-(j-1));
            end
        end
        for j=1:n_coef/2
            for i=j-1:n_order
                M_k(n_coef/2+j,i+1)=factorial(i)/factorial(i-(j-1))*ts(k)^(i-(j-1));
            end
        end
        M = blkdiag(M, M_k);
    end
end