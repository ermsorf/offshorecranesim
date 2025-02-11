export function rk4Step(Q, M, N, F, dt) {
  const n = Q.length;
  const M_inv = math.inv(M); // M⁻¹

  // Convert Q to a column state vector
  const q = Q.map(row => row[0]);  // Positions
  const qd = Q.map(row => row[1]); // Velocities
  const x = math.reshape(q.concat(qd), [2 * n, 1]);

  function computeXdot(x) {
    const qdot = math.reshape(x.slice(n, 2 * n), [n, 1]);
    const qddot = math.multiply(M_inv, math.subtract(F, math.multiply(N, qdot))); // [2,1]

    // Ensure x.slice(0, n) is also [2,1]
    const q = math.reshape(x.slice(0, n), [n, 1]);

    return math.concat(q, qddot, 0); // Concatenate along rows (axis=0)
  }

  const k1 = math.multiply(computeXdot(x), dt);
  const k2 = math.multiply(computeXdot(math.add(x, math.multiply(k1, 0.5))), dt);
  const k3 = math.multiply(computeXdot(math.add(x, math.multiply(k2, 0.5))), dt);
  const k4 = math.multiply(computeXdot(math.add(x, k3)), dt);

  const x_new = math.add(x, math.multiply(math.add(math.add(k1, math.multiply(k2, 2)), math.add(math.multiply(k3, 2), k4)), 1 / 6));

  return x_new.slice(0, n).map((q, i) => [q[0], x_new[n + i][0]]);
}
/*
export function rk4StepX(X, B, qdot, dt) {
  console.log("X",math.multiply(B,qdot))
  // Assuming computeXd depends on an updated qdot, or even X if needed.
  function computeXd(B, current_qdot) {
    return math.multiply(B, current_qdot);
  }

  const k1 = math.multiply(computeXd(B, qdot), dt);
  console.log("k1",k1)
  const k2 = math.multiply(computeXd(B, math.add(qdot, math.multiply(k1, 0.5))), dt);
  const k3 = math.multiply(computeXd(B, math.add(qdot, math.multiply(k2, 0.5))), dt);
  const k4 = math.multiply(computeXd(B, math.add(qdot, k3)), dt);

  return math.add(X, math.multiply(math.add(math.add(k1, math.multiply(k2, 2)),math.add(math.multiply(k3, 2), k4)), 1 / 6));
}
*/

export function rk4StepX(X, B, qDot, dt) {
  // RK4 helper function
  function f(B, qDot) {
    return math.multiply(B, qDot.map(parseFloat));
  }
  
  // RK4 steps
  let k1 = f(B, qDot);
  let qDot2 = qDot.map((qi, i) => parseFloat(qi) + (dt / 2) * k1[i]);
  let k2 = f(B, qDot2);
  let qDot3 = qDot.map((qi, i) => parseFloat(qi) + (dt / 2) * k2[i]);
  let k3 = f(B, qDot3);
  let qDot4 = qDot.map((qi, i) => parseFloat(qi) + dt * k3[i]);
  let k4 = f(B, qDot4);

  // Flatten X if it's a 2D array; otherwise, use X directly.
  const flatX = Array.isArray(X[0]) ? X.flat() : X;

  // Compute the RK4 summation for each element.
  const Xd = flatX.map((xi, i) => (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6);

  // Compute next state (flattened).
  const XNextFlat = flatX.map((xi, i) => xi + Xd[i] * dt);

  // Reassemble the flattened array into rows of length 6.
  const rowLength = 6;
  const XNext = [];
  for (let i = 0; i < XNextFlat.length; i += rowLength) {
    XNext.push(XNextFlat.slice(i, i + rowLength));
  }
  
  console.log("Xnext", XNext);
  return XNext;
}
