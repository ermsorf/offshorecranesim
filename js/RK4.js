
import { evaluateMatrix, computeTrigValues } from "./configimport.js";
import { system, variableMap, trigMap } from "./main_crane.js";
import { logQStep, logReactions, logPerformance } from "./csv.js";
import { stopSimulation, step, dt } from "./main_crane.js"; 
import { create, all } from 'https://cdn.jsdelivr.net/npm/mathjs@12.4.1/+esm';

const math = create(all);

export async function runRK4Step(system, Q, variableMap, trigMap, dt, controlForces, constants = {}) {
  if (!system) {
    console.error("System configuration not loaded.");
    return;
  }

  // Run RK4 step
  const newState = await rk4Step(system, Q, variableMap, trigMap, dt, controlForces, constants = {});

  if (
    newState.q.some(Number.isNaN) ||
    newState.qdot.some(Number.isNaN) ||
    newState.qddot.some(Number.isNaN)
  ) {
    stopSimulation();
    throw new Error("NaN detected in newState");
  }
  // Update Q
  const newQ = Q.map((row, i) => [
    newState.q[i],
    newState.qdot[i],
    newState.qddot[i],
  ]);

  let trigValues = computeTrigValues(newQ, variableMap, trigMap, constants);
  // console.log("trigValues", trigValues);
  let reactions = await evaluateMatrix(system.reactions, Q, variableMap, trigValues, constants);
  Q.splice(0, Q.length, ...newQ);
  logQStep(Q, system); // Log the Q step
  logReactions(reactions, system); // Log the reactions
}

export async function rk4Step(system, Q, variableMap, trigMap, dt, controlForces, constants = {}) {

  const n = Q.length;

  // Convert Q to a column state vector
  let q = Q.map((row) => row[0]); // Positions
  let qd = Q.map((row) => row[1]); // Velocities

  const x = math.reshape(q.concat(qd), [2 * n, 1]);

  
  async function computeXdot(x) {
    const q = math.reshape(x.slice(0, n), [n, 1]);
    const qdot = math.reshape(x.slice(n, 2 * n), [n, 1]);

    // Reshape x into Q format: [q1, q1d, 0; q2, q2d, 0; ...]
    const Q_new = Array.from({ length: n }, (_, i) => [q[i][0], qdot[i][0], 0]);
    // Evaluate matrices
    const trigValues = computeTrigValues(Q_new, variableMap, trigMap, constants);
    const [Mstar, Nstar, Bt_local, F_local] = await Promise.all([
      evaluateMatrix(system.Mstar, Q_new, variableMap, trigValues, constants),
      evaluateMatrix(system.Nstar, Q_new, variableMap, trigValues, constants),
      evaluateMatrix(system.Bt, Q_new, variableMap, trigValues, constants),
      evaluateMatrix(system.F, Q_new, variableMap, trigValues, constants),
    ]);

    
    
    const FstarGravity = math.multiply(Bt_local, F_local);
    // const FstarDamping = getDampingForces(Q_new, F_local, Bt_local);
    const FstarControl = getControlForces(Q_new, F_local, Bt_local, controlForces);
    const FstarTemp = math.add(FstarGravity, FstarControl); 


    const FstarConstraint = getConstraintForces(Q_new, Mstar, Nstar, FstarTemp);
    const FstarControlAfterConstraint = getControlAfterConstraintForces(Q_new, F_local, Bt_local, controlForces);
    const Fstar = math.add(FstarTemp, FstarConstraint, FstarControlAfterConstraint); 

    const Mstar_inv = math.inv(Mstar);

    const qddot = math.multiply(Mstar_inv, math.subtract(Fstar, math.multiply(Nstar, qdot)));

    return math.concat(qdot, qddot, 0); 
  }

  const k1 = math.multiply(await computeXdot(x), dt);
  const k2 = math.multiply(await computeXdot(math.add(x, math.multiply(k1, 0.5))), dt); // x = x + k1 * dt / 2
  const k3 = math.multiply(await computeXdot(math.add(x, math.multiply(k2, 0.5))), dt); // x = x + k2 * dt / 2
  const k4 = math.multiply(await computeXdot(math.add(x, k3)), dt);                     // x = x + k3 * dt

  const x_new = math.add(x, math.multiply(math.add(math.add(k1, math.multiply(k2, 2)), math.add(math.multiply(k3, 2), k4)),1 / 6));

  // Compute qddot at the final step
  let q_new = x_new.slice(0, n).map((v) => v[0]);
  let qdot_new = x_new.slice(n, 2 * n).map((v) => v[0]);
  let qddot_new_temp = await computeXdot(x_new);
  let qddot_new = qddot_new_temp.slice(n, 2 * n);
  return {
    q: q_new,
    qdot: qdot_new.map(val => val * 1), // Apply damping to qdot
    qddot: qddot_new.map((v) => v[0]),
  };
  }



function getConstraintForces(Q, Mstar, Nstar, Fstar) {
  const constraintCoordinates = Array.from({length: system.info.wiresegments }, (_, i) => `lambda${i + 1}`) // Selects all lambda coordinates to constrain
  // console.log("constraintCoordinates", constraintCoordinates)
  const n = Q.length;
  const k = constraintCoordinates.length;
  
  const A = constraintCoordinates.map(name => {
    const row = Array(n).fill(0);
    row[variableMap[name].i] = 1;
    return row;
  });
  const AT = math.transpose(A);

  const qd = math.reshape(Q.map(row => row[1]), [n, 1]);
  const rhsTop = math.subtract(Fstar, math.multiply(Nstar, qd));
  const rhs = math.concat(rhsTop, math.zeros([k, 1]), 0);

  const upper = math.concat(Mstar, AT, 1);
  const lower = math.concat(A, math.zeros([k, k]), 1);
  const K = math.concat(upper, lower, 0);

  const sol = math.lusolve(K, rhs);
  const lambda = sol.slice(n);
  const FstarConstraint = math.multiply(math.multiply(AT, lambda), -1);

  return FstarConstraint; 
}

function getControlForces(Q, F, Bt, controlForces) {
  let Fcontrol = new Array(F.length).fill().map(() => [0]);
  Fcontrol[6*0+5][0] = controlForces.BoomRotationZ
  Fcontrol[6*1+0][0] = controlForces.TrolleyTranslationX
  // Fcontrol[6*4+2][0] = 250

  let FstarControl = math.multiply(Bt, Fcontrol);
  return FstarControl; // Fstarcontrol
}
function getControlAfterConstraintForces(Q, F, Bt, controlForces) {
  let Fcontrol = new Array(F.length).fill().map(() => [0]);
  if (step*dt > 3) {
    // Fcontrol[6*4+2][0] = (Q[4][0] < -6 ? 10000 : -10000) //controlForces.TrolleyTranslationX
    };
  
  

  let FstarControlAfterConstraint = math.multiply(Bt, Fcontrol);
  return FstarControlAfterConstraint; // Fstarcontrol
}

function getDampingForces(Q, F, Bt) {

  let Fdamping = new Array(F.length).fill().map(() => [0]);
  Fdamping[5][0] = - 14700 * Q[0][1] // * math.abs(Q[0][1]); // Damping for Boom Rotation Z
  Fdamping[6][0] = - 1680 * Q[1][1]
  
  let ws = system.info.wiresegments; // # of wire segments
  if (ws >= 1) {
    Fdamping[6*2+3][0] = -200 * Q[2][1] * math.abs(Q[2][1]); // Damping for Wire 1 Rotation X
    Fdamping[6*3+5][0] = -200 * Q[3][0]; // Damping for Wire 1 Rotation Z
  }
  if (ws >= 2) {
    Fdamping[6*5+3][0] = -200 * Q[5][1] * math.abs(Q[5][1]); // Damping for Wire 2 Rotation X
    Fdamping[6*6+5][0] = -200 * Q[6][0]; // Damping for Wire 2 Rotation Z
  }
  if (ws >= 3) {
    Fdamping[6*8+3][0] = -200 * Q[8][1] * math.abs(Q[8][1]) ; // Damping for Wire 3 Rotation X
    Fdamping[6*9+5][0] = -200 * Q[9][0]; // Damping for Wire 3 Rotation Z
  }
  // Fdamping[Fdamping.length-1][0] = -5000 * Q[Q.length-1][1] * math.abs(Q[Q.length-1][1]); // Damping for Container Rotation Z

  let Fstardamping = math.multiply(Bt, Fdamping); 
  return Fstardamping;
}



