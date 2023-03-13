function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    % STEP 3.2.1 p constraint
    Aieq_p = zeros(2*(n_seg-1),n_all_poly);
    bieq_p = zeros(2*(n_seg-1),1);
    for k=1:n_seg-1
       Aieq_p(2*k-1,k*(n_order+1))=-1;
       Aieq_p(2*k,k*(n_order+1))=1;
       bieq_p(2*k-1,1)=-corridor_range(k,1);
       bieq_p(2*k,1)=corridor_range(k,2);
    end
    % STEP 3.2.2 v constraint   
    Aieq_v = zeros(2*(n_seg-1),n_all_poly);
    bieq_v = zeros(2*(n_seg-1),1);
    for k=1:n_seg-1
       Aieq_v(2*k-1,k*(n_order+1)-1)=-n_order;
       Aieq_v(2*k-1,k*(n_order+1))=n_order;
       bieq_v(2*k-1,1)=v_max;
       Aieq_v(2*k,k*(n_order+1)-1)=n_order;
       Aieq_v(2*k,k*(n_order+1))=-n_order;
       bieq_v(2*k,1)=v_max;
    end
    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros(2*(n_seg-1),n_all_poly);
    bieq_a = zeros(2*(n_seg-1),1);
    for k=1:n_seg-1
       Aieq_a(2*k-1,k*(n_order+1)-2)=n_order*(n_order-1);
       Aieq_a(2*k-1,k*(n_order+1)-1)=-2*n_order*(n_order-1);
       Aieq_a(2*k-1,k*(n_order+1))=n_order*(n_order-1);
       bieq_a(2*k-1,1)=a_max;
       Aieq_a(2*k,k*(n_order+1)-2)=-n_order*(n_order-1);
       Aieq_a(2*k,k*(n_order+1)-1)=2*n_order*(n_order-1);
       Aieq_a(2*k,k*(n_order+1))=-n_order*(n_order-1);
       bieq_a(2*k,1)=a_max;
    end
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
end