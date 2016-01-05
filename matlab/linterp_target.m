function pos = linterp_target(s, q)
n = size(q, 2);

% Form normalized chain
len = [];
chain = [];
for i = 1:n-1
    chain(:, i) = q(:, i+1) - q(:, i);
    len(i) = norm(chain(:, i));
end
chain = chain / sum(len);
len = len / sum(len);

% Find where in the chain we are (parameter s)
for i = 1:n-1
    if i == 1
        if s <= len(1) % between start and first point
            pos = s * chain(:, 1);
            break;
        end
    else
        if s > sum(len(1:i-1)) && s <= sum(len(1:i))+eps % between point i-1 and i
            s = (s - sum(len(1:i-1))) / len(i);
            pos = (1-s) * sum(chain(:, 1:i-1), 2) + s * sum(chain(:, 1:i), 2);
            break;
        end
    end
end

end

