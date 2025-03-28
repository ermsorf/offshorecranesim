
import { evaluateMatrix, computeTrigValues } from "./configimport.js";

export function rk4Step(Q, Mstar, Nstar, F, Bt, controlForces, dt) {
   // console.log("controlForces in rk4Step", controlForces);
  const n = Q.length;
  const M_inv = math.inv(Mstar); // M⁻¹

  // Convert Q to a column state vector
  const q = Q.map((row) => row[0]); // Positions
  const qd = Q.map((row) => row[1]); // Velocities
  const x = math.reshape(q.concat(qd), [2 * n, 1]);

  let Fconstraint = applyConstraintForces(M_inv, Mstar, Nstar, F, Bt, qd);
  let Fcontrol = getControlForces(F, Bt, controlForces);
  let Fstar = math.add(Fconstraint, Fcontrol);
    // let Fstar = math.multiply(Bt, F);

  function computeXdot(x) {
    const qdot = math.reshape(x.slice(n, 2 * n), [n, 1]);
    const qddot = math.multiply(M_inv, math.subtract(Fstar, math.multiply(Nstar, qdot))); // [n,1]
    const q = math.reshape(x.slice(0, n), [n, 1]);
    return math.concat(qdot, qddot, 0); // Concatenate along rows (axis=0)
  }

  const k1 = math.multiply(computeXdot(x), dt);
  const k2 = math.multiply(computeXdot(math.add(x, math.multiply(k1, 0.5))), dt);
  const k3 = math.multiply(computeXdot(math.add(x, math.multiply(k2, 0.5))), dt);
  const k4 = math.multiply(computeXdot(math.add(x, k3)), dt);

  const x_new = math.add(x, math.multiply(math.add(math.add(k1, math.multiply(k2, 2)), math.add(math.multiply(k3, 2), k4)),1 / 6));

  // Compute qddot at the final step
  const q_new = x_new.slice(0, n).map((v) => v[0]);
  const qdot_new = x_new.slice(n, 2 * n).map((v) => v[0]);
  const qddot_new = computeXdot(x_new).slice(n, 2 * n);

  // Return q, qdot, and qddot
  return {
    q: q_new,
    qdot: qdot_new,
    qddot: qddot_new.map((v) => v[0]),
  };
  }

export async function runRK4Step(system, Q, variableMap, trigMap, dt, controlForces) {
  if (!system) {
    console.error("System configuration not loaded.");
    return;
  }
  const trigValues = computeTrigValues(Q, variableMap, trigMap);

  const [Mstar, Nstar, Bt, F] = await Promise.all([
    evaluateMatrix(system.Mstar, Q, variableMap, trigValues),
    evaluateMatrix(system.Nstar, Q, variableMap, trigValues),
    evaluateMatrix(system.Bt, Q, variableMap, trigValues),
    evaluateMatrix(system.F, Q, variableMap, trigValues),
  ]);
  if (!Mstar || !Nstar || !Bt || !F) {
    console.error("Matrix evaluation failed. Aborting RK4 step.");
    return;
  }
  // Run RK4 step
  const newState = rk4Step(Q, Mstar, Nstar, F, Bt, controlForces, dt);

  // Update Q
  const newQ = Q.map((row, i) => [
    newState.q[i],
    newState.qdot[i],
    newState.qddot[i],
  ]);
  Q.splice(0, Q.length, ...newQ);
}

function applyConstraintForces(M_inv, Mstar, Nstar, F, Bt, qd) {
  const cancelCoordinates = [0,0, 0,0,1,  0,0,1,  0,0,1,  0]
  
  let Fstar = math.multiply(Bt, F);
  let qdCol = math.reshape(qd, [qd.length, 1]);
  let qdd = math.multiply(M_inv, math.subtract(Fstar, math.multiply(Nstar, qdCol)))
  console.log("qdd", qdd)
  let qdzero = qd.map((value, index) => cancelCoordinates[index] === 0 ? value : 0);
  let qddzero = qdd.map((value, index) => cancelCoordinates[index] === 0 ? value[0] : 0);

  qdzero = math.reshape(qdzero, [qd.length, 1]);
  qddzero = math.reshape(qddzero, [qdd.length, 1]); // Reshape qddzero to a column vector
  console.log("qddzero",qddzero)
  console.log("qdzero", qdzero)
  // let Fcontrol = math.multiply

  let Fconstraint = math.add(math.multiply(Mstar, qddzero), math.subtract(math.multiply(Nstar, qdzero), Fstar));
  console.log(Fconstraint)
  return Fconstraint
}

function getControlForces(F, Bt, controlForces) {
  let Fcontrol = new Array(F.length).fill([0]);
  Fcontrol[5][0] = controlForces.BoomRotationZ; 
  console.log("BRZ", controlForces.BoomRotationZ)
  console.log("TTX", controlForces.TrolleyTranslationX)
  Fcontrol[6][0] = controlForces.TrolleyTranslationX;

return math.multiply(Bt, Fcontrol);
}
