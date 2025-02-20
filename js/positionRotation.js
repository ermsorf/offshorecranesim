import { evaluateMatrix } from './configImport.js'

export async function getNextPos(xState, system, Q, variableMap, dt) {
    const qd = math.matrix(Q.map(row => [parseFloat(row[1])]));  // 6N × 1 column vector
    const qdd = math.matrix(Q.map(row => [parseFloat(row[2])])); // 6N × 1 column vector

    let B = await evaluateMatrix(system.B, Q, variableMap);
    let Bdot = await evaluateMatrix(system.Bdot, Q, variableMap);

    let Xdot = math.multiply(B, qd).toArray();   // Convert result to array
    let Xddot = math.multiply(Bdot, qdd).toArray();

    //console.log("Xdot full (6N×1)", Xdot);
    //console.log("Xddot full (6N×1)", Xddot);

    // Extract only the x1, x2, x3 components from every 6 rows (skip rotations)
    let Xdot_filtered = [];
    let Xddot_filtered = [];
    for (let i = 0; i < Xdot.length; i += 6) {
        Xdot_filtered.push([Xdot[i][0], Xdot[i + 1][0], Xdot[i + 2][0]]);
        Xddot_filtered.push([Xddot[i][0], Xddot[i + 1][0], Xddot[i + 2][0]]);
    }

    //console.log("Xdot filtered (only x values)", Xdot_filtered);
    //console.log("Xddot filtered (only x values)", Xddot_filtered);

    function updatePosition(xState, Xdot, Xddot, dt) {
        return xState.map((row, i) => 
            row.map((x, j) => x + Xdot[i][j] * dt + 0.5 * Xddot[i][j] * dt * dt)
        );
    }

    let xState_new = updatePosition(xState, Xdot_filtered, Xddot_filtered, dt);
    //console.log("Updated xState", xState_new);

    return xState_new;
}




export async function getRotationMatrices(system, Q, variableMap) {
    let rotations = system.rotations
    let cumulativeMatrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]; // Identity matrix
    const rotationMatrices = []; // Stores cumulative rotations

    rotations.forEach(({ axis, vars }) => {
        if (axis !== 0) { // Only apply rotation if axis ≠ 0
            const index = variableMap[vars];
            if (!index) throw new Error(`Variable ${vars} not found in Q`);

            const theta = Q[index.i][index.j]; // Retrieve theta value
            const R = getRotationMatrix(axis, theta); // Get incremental rotation
            cumulativeMatrix = math.multiply(cumulativeMatrix, R); // Multiply cumulatively
        }
        rotationMatrices.push(cumulativeMatrix); // Store result
    });

    return rotationMatrices;
}

    

