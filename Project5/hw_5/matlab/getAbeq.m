function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    Aeq_start(1,1)=1.0;
    Aeq_start(2,2)=1.0;
    Aeq_start(3,3)=2.0;
    Aeq_start(4,4)=6.0;
    beq_start=start_cond';
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    for k=1:4
        for i=k-1:n_order
            idx=i+1+(n_seg-1)*(n_order+1);
            Aeq_end(k,idx)=factorial(i)/factorial(i-(k-1))*(ts(end)^(i-(k-1)));
        end
    end
    beq_end=end_cond';
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    for k=1:(n_seg-1)
        for i=0:n_order 
            idx=i+1+(k-1)*(n_order+1);
            Aeq_wp(k,idx)=ts(k)^(i);
        end
    end
    beq_wp=waypoints(2:end-1);
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    for k=1:(n_seg-1)
        for i=0:n_order
            idx=(k-1)*(n_order+1)+i+1;
            Aeq_con_p(k,idx)=ts(k)^(i);
            Aeq_con_p(k,idx+n_order+1)=-0^(i);
        end
    end
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    for k=1:(n_seg-1)
        for i=1:(n_order)
            idx=(k-1)*(n_order+1)+i+1;
            Aeq_con_v(k,idx)=factorial(i)/factorial(i-1)*ts(k)^(i-1);
            Aeq_con_v(k,idx+n_order+1)=-factorial(i)/factorial(i-1)*0^(i-1);
        end
    end
    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    for k=1:(n_seg-1)
        for i=2:n_order
            idx=(k-1)*(n_order+1)+i+1;
            Aeq_con_a(k,idx)=factorial(i)/factorial(i-2)*ts(k)^(i-2);
            Aeq_con_a(k,idx+n_order+1)=-factorial(i)/factorial(i-2)*0^(i-2);
        end
    end
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    for k=1:(n_seg-1)
        for i=3:n_order
            idx=(k-1)*(n_order+1)+i+1;
            Aeq_con_j(k,idx)=factorial(i)/factorial(i-3)*ts(k)^(i-3);
            Aeq_con_j(k,idx+n_order+1)=-factorial(i)/factorial(i-3)*0^(i-3);
        end
    end
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end