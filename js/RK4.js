import { evaluateMatrix } from "./configimport.js";

export function rk4Step(Q, M, N, F, dt, damping = 0.996) {
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
    qdot: qdot_new.map(val => val * damping),
    qddot: qddot_new.map(v => v[0])
  };
}

export async function runRK4Step(system, Q, variableMap, dt) {
  if (!system) {
      console.error("System configuration not loaded.");
      return;
  }

  const startTotal = performance.now();

  try {
      const startEval = performance.now();
      const [Mstar, Nstar, Bt, F] = await Promise.all([
          evaluateMatrix(system.Mstar, Q, variableMap),
          evaluateMatrix(system.Nstar, Q, variableMap),
          evaluateMatrix(system.Bt, Q, variableMap),
          evaluateMatrix(system.F, Q, variableMap)
      ]);
      const endEval = performance.now();
      console.log(`Matrix evaluation: ${(endEval - startEval).toFixed(3)} ms`);

      if (!Mstar || !Nstar || !Bt || !F) {
          console.error("Matrix evaluation failed. Aborting RK4 step.");
          return;
      }

      const Fstar = math.multiply(Bt, F);

      const startRK4 = performance.now();
      const newState = rk4Step(Q, Mstar, Nstar, Fstar, dt);
      const endRK4 = performance.now();
      console.log(`RK4 Step: ${(endRK4 - startRK4).toFixed(3)} ms`);

      console.log("newState:", newState);
      
      // Atomic Q update
      const newQ = Q.map((row, i) => [
          newState.q[i], 
          newState.qdot[i], 
          newState.qddot[i]
      ]);
      Q.splice(0, Q.length, ...newQ);

  } catch (error) {
      console.error("Error in runRK4Step:", error);
  }

  const endTotal = performance.now();
  console.log(`Simulation Step: ${(endTotal - startTotal).toFixed(3)} ms`);
}

