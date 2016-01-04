target = [0 0.2 1.0; 
          0 0.8 2; 
          0 -0.6   0];

close all;figure;hold on;axis([-1 1 -1 1 -1 1]);
tpos = [];
ninter = 100; % Points to use for drawing chains
for i = 1:ninter
    tpos(:, i) = linterp_target(i/ninter, target);
end
plot3(tpos(1, :), tpos(2, :), tpos(3, :));

q0 = [0 0 0 0 0];
lb = [-0.3 -1.9 -1.9 -1.5 -1.7]; % lower bounds
ub = [1.3 1.9 1.9 -0.03 1.7]; % upper bounds

npts = 50; % Points to use for cost function
minobj = @(q)cost(@leftarmnormalized, q, target, npts);
opts = optimoptions(@fmincon, 'TolFun', 1e-1, 'TolCon', 1e-2,  'Display', 'off');
tic
[sol,~,~,output] = fmincon(minobj, q0, [],[],[],[], lb, ub, [], opts);
toc
draw_chain(@leftarmnormalized, sol);