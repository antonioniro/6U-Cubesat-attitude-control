function omega_next = propagate_omega_rk4(omega, J, dt)
% Propaga la velocità angolare omega usando Runge-Kutta 4° ordine
% secondo le equazioni di Eulero: dω/dt = J⁻¹ * ( -ω×(Jω) )

    % Funzione derivativa: equazioni di Eulero
    f = @(w) J \ ( -cross(w, J*w ) );

    k1 = f(omega);
    k2 = f(omega + 0.5*dt*k1);
    k3 = f(omega + 0.5*dt*k2);
    k4 = f(omega + dt*k3);

    omega_next = omega + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end