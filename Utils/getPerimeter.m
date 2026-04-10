function [perimeter] = getPerimeter(pts)
    assert((size(pts,1) == 2) || (size(pts,1) == 3), 'Input pointcloud <pts> must be [2 x N] or [3 x N]');

    perimeter = 0;
    for ii = 1:(size(pts,2)-1)
        perimeter = perimeter + norm(pts(:, ii) - pts(:, ii+1), 2);
    end
end