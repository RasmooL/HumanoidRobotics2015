% Load the MSRDaily txt files into cell arrays of matrices
% {frames} : 4x[x y z]'
function [left_arm, right_arm] = load_msr(fname)
f = fopen(fname, 'r');
header = textscan(f, '%d %d', 1); % nframes and nskele

left_arm = cell(header{1}, 1);
right_arm = cell(header{1}, 1);

for i = 1:header{1}
    ndata = textscan(f, '%d', 1);
    if ndata{1} ~= 40
        error('ndata in frame %d must be 40', i);
    end
    
    left_frame = zeros(3, 4); % fixed size because it's easy
    right_frame = zeros(3, 4); % fixed size because it's easy
    orig_pos = [];
    for j = 1:20
        pos = cell2mat(textscan(f, '%f %f %f %*d', 1));
        pos = [-pos(3); pos(1); pos(2)]; % fix coordinate system
        switch j
            case 5 % right arm
                orig_pos = pos;
                right_frame(:, 1) = pos - orig_pos;
            case 6
                right_frame(:, 2) = pos - orig_pos;
            case 7
                right_frame(:, 3) = pos - orig_pos;
            case 8
                right_frame(:, 4) = pos - orig_pos;
                
            case 9 % left arm
                orig_pos = pos;
                left_frame(:, 1) = pos - orig_pos;
            case 10
                left_frame(:, 2) = pos - orig_pos;
            case 11
                left_frame(:, 3) = pos - orig_pos;
            case 12
                left_frame(:, 4) = pos - orig_pos;
        end
        
        textscan(f, '%f %f %f %d', 1); % skip every 2nd data line, it's useless
    end
    right_arm{i} = right_frame;
    left_arm{i} = left_frame;
end