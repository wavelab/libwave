function y = measure(map, xr, Qi, rmax, thmax)
    flist = zeros(M, 1);
    
    for i = 1:M
        % if feature is visible
        if (is_feature_inview(map(:,i), xr(:,t), rmax, thmax))
            flist(i) = 1;
            
            % determine measurement
            d = gaussian_noise(Qi);
            y((2 * (i - 1) + 1: 2 * i), t) = [ ...
                sqrt((map(1,i)-xr(1,t))^2 + (map(2,i)-xr(2,t))^2);
                atan2(map(2,i)-xr(2,t),map(1,i)-xr(1,t))-xr(3,t) ...
            ] + d;
        end
    end
end