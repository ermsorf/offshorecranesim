import { evaluateMatrix, evaluateExpression } from "./configImport.js";

export function rk4Step(Q, M, N, F, dt) {
  const n = Q.length;
  const M_inv = math.inv(M); // M⁻¹

  // Convert Q to a column state vector
  const q = Q.map(row => row[0]);  // Positions
  const qd = Q.map(row => row[1]); // Velocities
  const x = math.reshape(q.concat(qd), [2 * n, 1]);

  function computeXdot(x) {
    const qdot = math.reshape(x.slice(n, 2 * n), [n, 1]);
    const qddot = math.multiply(M_inv, math.subtract(F, math.multiply(N, qdot))); // [n,1]
    const q = math.reshape(x.slice(0, n), [n, 1]);
    return math.concat(qdot, qddot, 0); // Concatenate along rows (axis=0)
  }

  const k1 = math.multiply(computeXdot(x), dt);
  const k2 = math.multiply(computeXdot(math.add(x, math.multiply(k1, 0.5))), dt);
  const k3 = math.multiply(computeXdot(math.add(x, math.multiply(k2, 0.5))), dt);
  const k4 = math.multiply(computeXdot(math.add(x, k3)), dt);

  const x_new = math.add(x, math.multiply(math.add(math.add(k1, math.multiply(k2, 2)), math.add(math.multiply(k3, 2), k4)), 1 / 6));

  // Compute qddot at the final step
  const q_new = x_new.slice(0, n).map(v => v[0]);
  const qdot_new = x_new.slice(n, 2 * n).map(v => v[0]);
  const qddot_new = math.multiply(M_inv, math.subtract(F, math.multiply(N, math.reshape(qdot_new, [n, 1]))));

  // Return q, qdot, and qddot
  return {
    q: q_new,
    qdot: qdot_new,
    qddot: qddot_new.map(v => v[0])
  };
}

export function runRK4Step(systemConfig, qState, dt) {
  if (!systemConfig) {
      console.error("System configuration not loaded.");
      return;
  }
  const { system } = systemConfig;
  const Q = evaluateMatrix(system.Qcoordinates, qState);
  const Mstar = evaluateMatrix(system.Mstar, qState);
  const Nstar = evaluateMatrix(system.Nstar, qState);
  const Bt = evaluateMatrix(system.Bt, qState);
  const F = evaluateMatrix(system.F, qState);
  const Fstar = math.multiply(Bt, F);

  // RK4 step
  const newState = rk4Step(Q, Mstar, Nstar, Fstar, dt);

  // Update qState
  system.Qcoordinates.forEach((qRow, i) => {
      qState[qRow[0].vars] = newState[i][0];
      qState[qRow[1].vars] = newState[i][1];
      qState[qRow[2].vars] = (newState[i][1] - Q[i][1]) / dt;
  });

  const updatedMatrices = Object.fromEntries(
      Object.entries(system)
        .filter(([key]) => key !== "Qcoordinates" && key !== "initconditions")
        .map(([key, value]) => [key, evaluateMatrix(value, qState)])
  );
  console.log("Updated Matrices:", updatedMatrices);
  console.log("qstate", qState);
}