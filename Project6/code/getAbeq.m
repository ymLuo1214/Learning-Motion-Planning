function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    Aeq_start = zeros(3,n_seg*(n_order+1)); 
    beq_start = [];
    Aeq_start(1,1)=1;
    Aeq_start(2,1)=-n_order;
    Aeq_start(2,2)=n_order;
    Aeq_start(3,1)=n_order*(n_order-1);
    Aeq_start(3,2)=-2*n_order*(n_order-1);
    Aeq_start(3,3)=n_order*(n_order-1);
    beq_start=start_cond';

    Aeq_end =zeros(3,n_seg*(n_order+1)); 
    beq_end = [];
    Aeq_end(1,end)=1;
    Aeq_end(2,end-1)=-n_order;
    Aeq_end(2,end)=n_order;
    Aeq_end(3,end-2)=n_order*(n_order-1);
    Aeq_end(3,end-1)=-2*n_order*(n_order-1);
    Aeq_end(3,end)=n_order*(n_order-1);
    beq_end=end_cond';

    Aeq_con_p =zeros(n_seg-1,n_seg*(n_order+1));
    beq_con_p = zeros(n_seg-1,1);
    for k=1:n_seg-1
        Aeq_con_p(k,k*(n_order+1))=1;
        Aeq_con_p(k,k*(n_order+1)+1)=-1;
    end

    Aeq_con_v = zeros(n_seg-1,n_seg*(n_order+1));
    beq_con_v =zeros(n_seg-1,1);
    for k=1:n_seg-1
        Aeq_con_v(k,k*(n_order+1)-1)=-n_order;
        Aeq_con_v(k,k*(n_order+1))=n_order;
        Aeq_con_v(k,k*(n_order+1)+1)=n_order;
        Aeq_con_v(k,k*(n_order+1)+2)=-n_order;
    end

    Aeq_con_a = zeros(n_seg-1,n_seg*(n_order+1));
    beq_con_a =zeros(n_seg-1,1);
    for k=1:n_seg-1
        Aeq_con_a(k,k*(n_order+1)-2)=n_order*(n_order-1);
        Aeq_con_a(k,k*(n_order+1)-1)=-2*n_order*(n_order-1);
        Aeq_con_a(k,k*(n_order+1))=n_order*(n_order-1);
        Aeq_con_a(k,k*(n_order+1)+1)=-n_order*(n_order-1);
        Aeq_con_a(k,k*(n_order+1)+2)=2*n_order*(n_order-1);
        Aeq_con_a(k,k*(n_order+1)+3)=-n_order*(n_order-1);
    end

    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end