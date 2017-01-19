function noise = gaussian_noise(sigma)
    [SE, Se] = eig(sigma);
    noise = SE * sqrt(Se) * randn(length(sigma), 1);
end
