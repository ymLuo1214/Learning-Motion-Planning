function Q = getQ(n_seg, n_order, ts)
    Q = [];
    n_coef=n_order+1;
    for k = 1:n_seg
        Q_k =zeros(n_order+1,n_order+1);
        for i = 5:n_coef
            for j=5:n_coef
                Q_k(i,j)=(i-1)*(i-2)*(i-3)*(i-4)*(j-1)*(j-2)*(j-3)*(j-4)*ts(k)^(i+j-9)/(i+j-9);
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end