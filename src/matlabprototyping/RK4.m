function [systemStateNew] = RK4(systemState, t, dt, equation)

    k1 = dt * f(systemState, t)
    k2 = dt * f(systemState + 0.5 * k1, t + 0.5 * dt)
    k3 = dt * f(systemState + 0.5 * k2, t + 0.5 * dt)
    k4 = dt * f(systemState + k3, t + dt)
    systemStateNew = systemState + (k1 + 2*k2 + 2*k3 + k4) / 6
end