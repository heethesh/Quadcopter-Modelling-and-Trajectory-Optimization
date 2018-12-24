function mat = mat2row(mat, r, c)
    [r_, c_, n] = size(mat);
    mat = mat(r, c, :);
    mat = reshape(mat, 1, n);
end