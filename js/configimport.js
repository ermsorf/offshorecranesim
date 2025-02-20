export async function loadSystemConfig(filePath) {
    const response = await fetch(filePath);
    const system = await response.json();

    // Create variableMap to track indices of variables in Q. Like a lookup map. theta1 value stored in Q[0][0]
    let variableMap = {};
    system.Qcoordinates.forEach((row, i) => {
        row.forEach((entry, j) => {
            if (entry.vars) {
                variableMap[entry.vars] = { i, j };
            }
        });
    });

    // Initialize empty Q matrix
    let Q = system.Qcoordinates.map(row => row.map(() => 0));

    // Now populate Q with evaluated values
    for (let i = 0; i < system.Qcoordinates.length; i++) {
        for (let j = 0; j < system.Qcoordinates[i].length; j++) {
            Q[i][j] = evaluateExpression(system.initconditions[i][j], Q, variableMap);
        }
    }

    console.log("Initial Q:", Q);
    return { system, Q, variableMap };
}
export function evaluateExpression(entry, Q, variableMap) {
    if (!entry || typeof entry.expr !== "string") {
        console.warn("Invalid entry:", entry);
        return NaN;
    }

    const { expr, vars } = entry;
    const variables = Array.isArray(vars) ? vars : vars?.trim() ? [vars] : [];

    if (variables.length === 0) {
        try {
            return new Function(`return ${expr};`)();
        } catch (e) {
            console.error("Eval failed for constant expression:", expr, "Error:", e);
            return NaN;
        }
    }

    const missingVars = variables.filter(v => !(v in variableMap));
    if (missingVars.length > 0) {
        console.error("Missing variables in Q:", missingVars);
        return NaN;
    }

    try {
        // Retrieve values from Q using the variable map
        const values = variables.map(v => {
            let { i, j } = variableMap[v];
            return Q[i][j];
        });

        return new Function(...variables, `return ${expr};`)(...values);
    } catch (e) {
        console.error("Eval failed for:", expr, "Error:", e);
        return NaN;
    }
}

export function evaluateMatrix(matrix, Q, variableMap) {
    if (!Array.isArray(matrix)) return evaluateExpression(matrix, Q, variableMap);
    return matrix.map(row =>
        Array.isArray(row) ? row.map(entry => evaluateExpression(entry, Q, variableMap)) : evaluateExpression(row, Q, variableMap)
    );
}
