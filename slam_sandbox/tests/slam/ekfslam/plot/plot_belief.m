function plot_belief(x, t)
    plot(x(1, 1:t), x(2, 1:t), 'bo--');
    plot( ...
        [x(1, t), x(1, t) + 1 * cos(x(3, t))], ...
        [x(2, t), x(2, t) + 1 * sin(x(3, t))], ...
        'b-' ...
    );
end