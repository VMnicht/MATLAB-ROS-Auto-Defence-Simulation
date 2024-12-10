function V=limitV(maxV,targetV,acc,dt)
    new_velocity = targetV + acc * dt;
    speed_magnitude = norm(new_velocity);
    V = speed_magnitude;
    if speed_magnitude > maxV
        % 缩放速度
        scaling_factor = maxV / speed_magnitude;
        V = new_velocity * scaling_factor;
    end
end