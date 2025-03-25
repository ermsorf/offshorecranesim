function dydt = pendulumODE(t, y, l, g)
    % y(1) = theta
    % y(2) = thetadot
    % y(3) = phi
    % y(4) = phidot

    % Extract state variables
    theta = y(1);
    thetadot = y(2);
    phi = y(3);
    phidot = y(4);

    % Precompute trigonometric terms
    sin2phi = sin(2 * phi);
    sin_phi = sin(phi);

    % Compute second derivatives (d^2theta/dt^2 and d^2phi/dt^2)
    denom1 = 5000 * l^2 * sin_phi^2 + 2;
    denom2 = 5000 * l^2 + 1;

    thetaddot = (-5000 * l^2 * thetadot * phidot * sin2phi) / denom1;
    phiddot = (5000 * l * (l * sin2phi * thetadot^2 / 2 + g * sin_phi)) / denom2;

    % Define first-order system
    dydt = [thetadot; thetaddot; phidot; phiddot]; % No dl/dt since l is constant
end