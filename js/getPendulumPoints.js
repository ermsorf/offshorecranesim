function Solve313(initialConditions) {
    // Parameters
    const l = initialConditions.length;
    const g = 9.81;
    
    // State vector: [theta1, theta1_dot, theta2, theta2_dot, theta3, theta3_dot]
    // Note: Adjust the initial conditions accordingly.
    const Y0 = [
      initialConditions.theta,       // theta1
      initialConditions.thetadot,    // theta1_dot
      initialConditions.phi,         // theta2
      initialConditions.phidot,      // theta2_dot
      initialConditions.psi,         // theta3
      initialConditions.psidot       // theta3_dot
    ];
    
    // Time span parameters
    const t0 = 0;
    const t1 = 5;
    const steps = 50;
    const dt = (t1 - t0) / (steps - 1);
    
    const tValues = new Array(steps);
    const YValues = new Array(steps);
    tValues[0] = t0;
    YValues[0] = Y0;
    
    // Integrate using RK4
    for (let i = 1; i < steps; i++) {
      const tPrev = tValues[i - 1];
      const YPrev = YValues[i - 1];
      const { t: tNew, Y: YNew } = rk4Step(pendulumODE, tPrev, YPrev, dt, l, g);
      tValues[i] = tNew;
      YValues[i] = YNew;
    }
    
    // Convert generalized coordinates to Cartesian coordinates
    // (Using the same conversion as before)
    const xPendulumFromPivot = [];
    const yPendulumFromPivot = [];
    const zPendulumFromPivot = [];
    
    for (let i = 0; i < steps; i++) {
      const Y = YValues[i];
      // Here we use: x = l*sin(phi)*cos(theta),
      //             y = l*sin(phi)*sin(theta),
      //             z = -l*cos(phi)
      // with: phi = theta2 and theta = theta1.
      const theta1 = Y[0];
      const phi = Y[2];
      const x = l * Math.sin(phi) * Math.cos(theta1);
      const y = l * Math.sin(phi) * Math.sin(theta1);
      const z = -l * Math.cos(phi);
      xPendulumFromPivot.push(x);
      yPendulumFromPivot.push(y);
      zPendulumFromPivot.push(z);
    }
    
    // Compute rotation matrices for each time step.
    // Using 3–1–3 Euler angles: R1 = theta1, R2 = -phi, R3 = psi.
    const R_all = [];
    for (let i = 0; i < steps; i++) {
      const Y = YValues[i];
      const R1 = Y[0];
      const R2 = -Y[2];
      const R3 = Y[4];
      const Rz1 = rotationZ(R1);
      const Ry = rotationY(R2);
      const Rz2 = rotationZ(R3);
      const R_temp = multiplyMatrices(Rz1, Ry);
      const R_final = multiplyMatrices(R_temp, Rz2);
      R_all.push(R_final);
    }
    
    return {
      xPendulumFromPivot,
      yPendulumFromPivot,
      zPendulumFromPivot,
      R: R_all,
      t: tValues,
      pivot: initialConditions.pivot
    };
  }
  
  // -----------------------------------
  // RK4 Solver Implementation (same as before)
  // -----------------------------------
  function rk4Step(odeFunc, t, Y, dt, l, g) {
    const k1 = odeFunc(t, Y, l, g);
    const Y2 = Y.map((y, i) => y + (dt / 2) * k1[i]);
    const k2 = odeFunc(t + dt / 2, Y2, l, g);
    const Y3 = Y.map((y, i) => y + (dt / 2) * k2[i]);
    const k3 = odeFunc(t + dt / 2, Y3, l, g);
    const Y4 = Y.map((y, i) => y + dt * k3[i]);
    const k4 = odeFunc(t + dt, Y4, l, g);
    
    const Y_next = Y.map((y, i) => y + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    return { t: t + dt, Y: Y_next };
  }
  
  // -----------------------------------
  // New ODE Function Using Your Equations
  // -----------------------------------
  // State: Y = [theta1, theta1_dot, theta2, theta2_dot, theta3, theta3_dot]
  function pendulumODE(t, Y, l, g) {
    const theta1    = Y[0];
    const theta1_dot = Y[1];
    const theta2    = Y[2];
    const theta2_dot = Y[3];
    const theta3    = Y[4];
    const theta3_dot = Y[5];
  
    // Pre-calculate common trigonometric terms:
    const sinTheta2 = Math.sin(theta2);
    const cosTheta2 = Math.cos(theta2);
    const sin2Theta2 = Math.sin(2 * theta2);
    const cos2Theta2 = Math.cos(2 * theta2);
  
    // --- Equation (B) ---
    // This equation (from your second equation) is squared to solve for theta2_ddot.
    // It has the form:
    //   (theta1_dot * Denom / 2)^2 = 1562500*sin(theta2)*theta3_dot + 2441406250000*sin(theta2)^2*theta3_dot^2
    //      + sin(theta2)*C*theta2_ddot + sin(theta2)*D
    // where:
    //   Denom = 3*(250000*sin(2*theta2)*l^2 - 1000*sin(2*theta2)*l + sin(2*theta2))
    //   C = 9375459*cos(theta2) - 9375468000*l*cos(theta2) + 2343876000000*l^2*cos(theta2)
    //       - 4500000000*l^3*cos(theta2) + 562500000000*l^4*cos(theta2)
    //   D = -6750*g*cos(theta2)*sin(theta2) - 3937500000*g*l^2*cos(theta2)*sin(theta2)
    //       + 562500000000*g*l^3*cos(theta2)*sin(theta2) + 9000000*g*l*cos(theta2)*sin(theta2)
    const denom = 3 * (250000 * sin2Theta2 * l * l - 1000 * sin2Theta2 * l + sin2Theta2);
    const LHS = Math.pow(theta1_dot * (denom / 2), 2);
    const C = 9375459 * cosTheta2 - 9375468000 * l * cosTheta2 + 2343876000000 * l * l * cosTheta2
              - 4500000000 * Math.pow(l, 3) * cosTheta2 + 562500000000 * Math.pow(l, 4) * cosTheta2;
    const D = -6750 * g * cosTheta2 * sinTheta2 - 3937500000 * g * l * l * cosTheta2 * sinTheta2
              + 562500000000 * g * Math.pow(l, 3) * cosTheta2 * sinTheta2 + 9000000 * g * l * cosTheta2 * sinTheta2;
    
    // Solve for theta2_ddot:
    // Rearranging the squared relation gives:
    //   theta2_ddot = (LHS - 1562500*sinTheta2*theta3_dot - 2441406250000*sinTheta2*sinTheta2*theta3_dot^2 - sinTheta2*D)
    //                 / (sinTheta2 * C)
    const theta2_ddot = (LHS - 1562500 * sinTheta2 * theta3_dot - 2441406250000 * sinTheta2 * sinTheta2 * theta3_dot * theta3_dot - sinTheta2 * D) / (sinTheta2 * C);
  
    // --- Equation (C) ---
    // Your third equation (after substituting the relation from the first equation)
    // leads to an expression for theta1_ddot:
    // Define coefficients A1 and B1:
    const A1 = 750000 * l * l - 3 * cos2Theta2 - 3000 * l - 6250000 * (cosTheta2 * cosTheta2)
               - 750000 * l * l * cos2Theta2 + 6250603 + 3000 * l * cos2Theta2;
    const B1 = 6250000 * cosTheta2 * theta2_dot * sinTheta2 * theta1_dot
               + 6 * Math.sin(2 * theta2) * theta1_dot * theta2_dot
               - 6000 * l * Math.sin(2 * theta2) * theta1_dot * theta2_dot
               + 1500000 * l * l * Math.sin(2 * theta2) * theta1_dot * theta2_dot;
    
    // Then:
    //   theta1_ddot = (6250000*sinTheta2*theta2_dot*theta3_dot - B1) / A1
    const theta1_ddot = (6250000 * sinTheta2 * theta2_dot * theta3_dot - B1) / A1;
    
    // --- Equation (A) ---
    // The first equation gives:
    //   theta3_ddot = theta2_dot*sinTheta2*theta1_dot - cosTheta2*theta1_ddot
    const theta3_ddot = theta2_dot * sinTheta2 * theta1_dot - cosTheta2 * theta1_ddot;
  
    // Assemble the derivative vector:
    return [
      theta1_dot,      // d(theta1)/dt
      theta1_ddot,     // d(theta1_dot)/dt
      theta2_dot,      // d(theta2)/dt
      theta2_ddot,     // d(theta2_dot)/dt
      theta3_dot,      // d(theta3)/dt
      theta3_ddot      // d(theta3_dot)/dt
    ];
  }
  
  // -----------------------------------
  // Rotation Matrix Functions (same as before)
  // -----------------------------------
  
  // Rotation about Z-axis
  function rotationZ(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [
      [c, -s, 0],
      [s,  c, 0],
      [0,  0, 1]
    ];
  }
  
  // Rotation about Y-axis
  function rotationY(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [
      [ c, 0, s],
      [ 0, 1, 0],
      [-s, 0, c]
    ];
  }
  
  // Multiply two 3x3 matrices (A * B)
  function multiplyMatrices(A, B) {
    const result = [];
    for (let i = 0; i < 3; i++) {
      result[i] = [];
      for (let j = 0; j < 3; j++) {
        result[i][j] = 0;
        for (let k = 0; k < 3; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }
    return result;
  }
  