function draw_chain(chain, q, offset)
    p = [];
    num = 100;
    for i = 1:num
        p(:, i) = chain(i/num, q(1), q(2), q(3), q(4), q(5)) + offset;
    end
    plot3(p(1, :), p(2, :), p(3, :));
end