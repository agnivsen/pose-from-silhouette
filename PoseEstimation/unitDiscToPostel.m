function [alpha, rVector] = unitDiscToPostel(X, Y)
    alpha = X.^2 + Y.^2;
    rVector = [X Y zeros(numel(X), 1)];
    rVector = normc(rVector.').';
end