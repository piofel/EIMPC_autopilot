function a = unwrap_angle(a, a_prev)
    while a - a_prev > pi
        a = a - 2*pi;
    end
    while a - a_prev < -pi
        a = a + 2*pi;
    end
end