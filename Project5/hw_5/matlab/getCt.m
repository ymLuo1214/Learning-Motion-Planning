function Ct = getCt(n_seg, n_order)
    n_coef=n_order+1;
    n_status=n_coef/2;
    Ct=zeros(n_seg*n_coef,n_seg*n_coef-(n_seg-1)*n_status);
    Ct(1:n_status,1:n_status)=eye(n_status,n_status);
    Ct(1+end-n_status:end,n_status+1:n_status*2)=eye(n_status,n_status);
    %Position Continuity & Waypoint Position Constraints
    for k =1:n_seg-1
        Ct(1+(2*k-1)*n_status,n_coef+k)=1;
        Ct(1+(2*k)*n_status,n_coef+k)=1;
    end
    %Velocity Continuity Constraints
    for k =1:n_seg-1
        Ct(2+(2*k-1)*n_status,n_coef+n_seg-1+k)=1;
        Ct(2+(2*k)*n_status,n_coef+n_seg-1+k)=1;
    end
    %Acceleration Continuity Constraints
    for k =1:n_seg-1
        Ct(3+(2*k-1)*n_status,n_coef+2*(n_seg-1)+k)=1;
        Ct(3+(2*k)*n_status,n_coef+2*(n_seg-1)+k)=1;
    end
    %Jerk Continuity Constraints
    for k =1:n_seg-1
        Ct(4+(2*k-1)*n_status,n_coef+3*(n_seg-1)+k)=1;
        Ct(4+(2*k)*n_status,n_coef+3*(n_seg-1)+k)=1;
    end
end