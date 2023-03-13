clc;clear;close all;
path = ginput() * 100.0;

n_order       =7;% order of poly
n_seg         = size(path,1)-1;% segment number
n_poly_perseg = (n_order+1); % coef number of perseg

ts = zeros(n_seg, 1);
% calculate time distribution in proportion to distance between 2 points
dist     = zeros(n_seg, 1);
dist_sum = 0;
T        = 25;
t_sum    = 0;

for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum+dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;
    t_sum = t_sum+ts(i);
end
ts(n_seg) = T - t_sum;

poly_coef_x1 = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y1 = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);
poly_coef_x2 = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y2 = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);

% display the trajectory
X_n1 = [];
Y_n1 = [];
X_n2 = [];
Y_n2 = [];
k = 1;
tstep = 0.01;
hold on
for i=0:n_seg-1
    %#####################################################
    Pxi1=poly_coef_x1(i*(n_order+1)+1:i*(n_order+1)+1+n_order);
    Pyi1=poly_coef_y1(i*(n_order+1)+1:i*(n_order+1)+1+n_order);
    Pxi1=Pxi1(end:-1:1);
    Pyi1=Pyi1(end:-1:1);
    Pxi2=poly_coef_x2(i*(n_order+1)+1:i*(n_order+1)+1+n_order);
    Pyi2=poly_coef_y2(i*(n_order+1)+1:i*(n_order+1)+1+n_order);
    Pxi2=Pxi2(end:-1:1);
    Pyi2=Pyi2(end:-1:1);
    for t = 0:tstep:ts(i+1)
        if k<20
            X_n1(k)  = polyval(Pxi1, t);
            Y_n1(k)  = polyval(Pyi1, t);
        end
        if k>=20
            X_n2(k-19)  = polyval(Pxi2, t);
            Y_n2(k-19)  = polyval(Pyi2, t);
        end
        k = k+1;
        if k==21
            plot(X_n1, Y_n1 , 'Color', [0 1.0 0], 'LineWidth', 1);
            X_n1 = [];
            Y_n1 = [];
        end
        if k==41
            plot(X_n2, Y_n2, 'Color', [1.0 0 0], 'LineWidth', 1);
            k=1;
            X_n2 = [];
            Y_n2 = [];
        end
    end
end
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));