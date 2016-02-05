%% Init
close all;clear;clc;
addpath rvctools
run startup_rvc
run mdl_nao % I added the wrist and hand to the DH Nao model.

[left_targets, right_targets] = load_msr('a01_s01_e01_skeleton.txt');

ninter = 30; % Points to use for drawing chains
npts = 50; % Points to use for cost function
figure;hold on;axis([-1 1 -1 1 -1 1]); view(45, 20); xlabel('x'); ylabel('y'); zlabel('z');
%% Create animation
nframes = length(left_targets);
M(nframes) = struct('cdata', [], 'colormap', []);

% Keep previous solutions as starting point for this optimization.
% ~10x speed boost and improves smoothness of movement!
lsol = [0 0 0 -0.1 0];
rsol = [0 0 0 0.1 0];

for i = 1:nframes
    cla;
    %% Left arm
    target = left_targets{i};
    loffset = [0; 0.2; 0];

    tpos = [];
    for j = 1:ninter
        tpos(:, j) = linterp_target(j/ninter, target) + loffset;
    end
    plot3(tpos(1, :), tpos(2, :), tpos(3, :));

    lb = [-1.9 -0.3 -1.9 -1.5 -1.7]; % lower bounds
    ub = [1.9 1.3 1.9 -0.03 1.7]; % upper bounds

    minobj = @(q)cost(@leftarmnormalized, q, target, npts);
    opts = optimoptions(@fmincon, 'TolFun', 2e-1, 'TolCon', 1e-1,  'Display', 'off');
    tic
    [lsol,~,~,output] = fmincon(minobj, lsol, [],[],[],[], lb, ub, [], opts);
    toc
    draw_chain(@leftarmnormalized, lsol, loffset);
    
    %% Right arm
    target = right_targets{i};
    roffset = [0; -0.2; 0];

    tpos = [];
    for j = 1:ninter
        tpos(:, j) = linterp_target(j/ninter, target) + roffset;
    end
    plot3(tpos(1, :), tpos(2, :), tpos(3, :));

    lb = [-1.9 -1.3 -1.9 0.04 -1.7]; % lower bounds
    ub = [1.9 0.3 1.9 1.5 1.7]; % upper bounds

    minobj = @(q)cost(@rightarmnormalized, q, target, npts);
    opts = optimoptions(@fmincon, 'TolFun', 1e-1, 'TolCon', 1e-2,  'Display', 'off');
    %tic
    [rsol,~,~,output] = fmincon(minobj, rsol, [],[],[],[], lb, ub, [], opts);
    %toc
    draw_chain(@rightarmnormalized, rsol, roffset);
    
    %% Save to animation
    drawnow
    M(i) = getframe;
end

%% Save animation
movie2avi(M, 'anim.avi');