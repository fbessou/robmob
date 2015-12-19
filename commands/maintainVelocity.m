function maintainVelocity(twist, duration,publisher)
    for i = 0:0.01:duration
        send(publisher,twist);
        pause(0.01);
    end
end

