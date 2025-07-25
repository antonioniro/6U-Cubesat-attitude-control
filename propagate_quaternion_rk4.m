function q_next = propagate_quaternion_rk4(q, omega_body, dt)
% PROPAGATE_QUATERNION integra q̇ = ½Ω(ω)q con RK4
    wx=omega_body(1); wy=omega_body(2); wz=omega_body(3);

    % Omega = [  0    wz   -wy   wx;
    %           -wz   0     wx   wy;
    %            wy  -wx    0    wz;
    %           -wx  -wy   -wz    0 ];

    skew_mat = -[0 -omega_body(3) omega_body(2)        %Skew matrix for angular velocity quaternion multiplication (slide 5 of presentation lesson 2)  
                 omega_body(3) 0 -omega_body(1)
                -omega_body(2) omega_body(1) 0];
    
    f = @(q0) [(0.5*skew_mat*q0(1:3)+0.5*q0(4)*omega_body)',-0.5*omega_body'*q0(1:3)]';
    % RK4
    k1 = f(q);
    k2 = f(q + 0.5*dt*k1);
    k3 = f(q + 0.5*dt*k2);
    k4 = f(q +    dt*k3);

    q_next = q + dt/6*(k1+2*k2+2*k3+k4);

    % normalize
    q_next = q_next / norm(q_next);
end