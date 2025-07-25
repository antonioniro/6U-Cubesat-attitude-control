function q_next = propagate_quaternion(q, omega_body, dt)
% Propaga un quaternione q nel tempo usando la cinematica del quaternione
% q: quaternione corrente [4x1] (formato [qv; q4])
% omega_body: velocità angolare [rad/s] nel frame body [3x1]
% dt: intervallo di tempo [s]

    theta = norm(omega_body) * dt;

    if theta > 1e-8
        axis = omega_body / norm(omega_body);
        dq_v = sin(theta/2) * axis;
        dq_4 = cos(theta/2);
    else 
        dq_v = 0.5 * omega_body * dt;
        dq_4 = 1;
    end

    dq = [dq_v; dq_4]; %quaternione incrementale

    % : q_next = dq ⓧ q
    q_next = q_product(dq,q);

    % Rinormalizzazione
    q_next = q_next / norm(q_next);
end


