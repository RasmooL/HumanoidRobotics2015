function v = cost(chain, q, target, ninterp)
    v = 0;
    for i = 1:ninterp
        v = v + norm(chain(i/ninterp, q(1), q(2), q(3), q(4), q(5)) - linterp_target(i/ninterp, target))^2;
    end
end