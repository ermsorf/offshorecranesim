export async function loadSystemConfig(filePath) {
    const response = await fetch(filePath);
    const system = await response.json();
    console.log("Loaded system config:", system);

    // Initialize Q with initial conditions
    let Q = system.Qcoordinates.map((row, i) => [
        evaluateExpression(system.initconditions[i][0], Q, i, 0), // q
        evaluateExpression(system.initconditions[i][1], Q, i, 1), // qdot
        evaluateExpression(system.initconditions[i][2], Q, i, 2)  // qddot
    ]);

    console.log("Initial Q:", Q);
    return { Q, system };
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
