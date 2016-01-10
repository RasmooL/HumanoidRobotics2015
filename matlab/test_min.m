%% Init
addpath rvctools
run startup_rvc
run mdl_nao

ninter = 50; % Points to use for drawing chains
npts = 30; % Points to use for cost function

close all;figure;hold on;axis([-1 1 -1 1 -1 1]); view(45, 20); xlabel('x'); ylabel('y'); zlabel('z');

%% Right arm
target = [0  0.1   0.3; 
          0 -0.3  -0.5; 
          0  0.0  -0.1];

tpos = [];
for i = 1:ninter
    tpos(:, i) = linterp_target(i/ninter, target);
end
plot3(tpos(1, :), tpos(2, :), tpos(3, :));

q0 = [0 0 0 0.1 0];
lb = [-1.9 -1.3 -1.9 0.04 -1.7]; % lower bounds
ub = [1.9 0.3 1.9 1.5 1.7]; % upper bounds

minobj = @(q)cost(@rightarmnormalized, q, target, npts);
opts = optimoptions(@fmincon, 'TolFun', 1e-1, 'TolCon', 1e-2,  'Display', 'off');
tic
[sol,~,~,output] = fmincon(minobj, q0, [],[],[],[], lb, ub, [], opts);
toc
draw_chain(@rightarmnormalized, sol);

%% Left arm
target = [0  0.1   0.3; 
          0  0.3   0.5; 
          0  0.0  -0.1];

tpos = [];
for i = 1:ninter
    tpos(:, i) = linterp_target(i/ninter, target);
end
plot3(tpos(1, :), tpos(2, :), tpos(3, :));

q0 = [0 0 0 -0.1 0];
lb = [-1.9 -0.3 -1.9 -1.5 -1.7]; % lower bounds
ub = [1.9 1.3 1.9 -0.03 1.7]; % upper bounds

minobj = @(q)cost(@leftarmnormalized, q, target, npts);
opts = optimoptions(@fmincon, 'TolFun', 1e-1, 'TolCon', 1e-2,  'Display', 'off');
tic
[sol,~,~,output] = fmincon(minobj, q0, [],[],[],[], lb, ub, [], opts);
toc
draw_chain(@leftarmnormalized, sol);