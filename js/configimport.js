export async function loadSystemConfig(filePath) {
    const response = await fetch(filePath);
    const system = await response.json();

    let variableMap = {};
    system.Qcoordinates.forEach((row, i) => {
        row.forEach((entry, j) => {
            if (entry.vars) {
                variableMap[entry.vars] = { i, j };
            }
        });
    });

    // Step 1: Initialize empty Q matrix
    let Q = system.Qcoordinates.map(row => row.map(() => 0));

    // Step 2: Populate Q using already initialized variableMap
    for (let i = 0; i < system.Qcoordinates.length; i++) {
        for (let j = 0; j < system.Qcoordinates[i].length; j++) {
            Q[i][j] = await evaluateExpression(system.initconditions[i][j], Q, variableMap);
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

export async function evaluateMatrix(matrix, Q, variableMap) {
    if (!Array.isArray(matrix)) return evaluateExpression(matrix, Q, variableMap);
    return Promise.all(matrix.map(row =>
        Array.isArray(row) ? Promise.all(row.map(entry => evaluateExpression(entry, Q, variableMap))) : evaluateExpression(row, Q, variableMap)
    ));
}
