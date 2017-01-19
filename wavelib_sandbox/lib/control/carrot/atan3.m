
function angle = atan3(y, x)
    angle = atan2(y, x);

    if y < 0
        angle = angle + 2 * pi;
    end
end
