export async function loadSystemConfig() {
    const response = await fetch('../src/FrameGen/testconfig.json');
    const system = await response.json();
    console.log("Loaded system config:", system);

    let state = {};
    let Qcoordinates = system.Qcoordinates.map((row, i) =>
        row.map((entry, j) => {
            const key = entry.expr;
            const value = evaluateExpression(system.initconditions[i][j], state);
            state[key] = value;
            return value;
        })
    );

    console.log("Initial Conditions:", state);

    console.time("Matrix Evaluation");
    let evaluatedMatrices = {
        Qcoordinates,
        ...Object.fromEntries(
            Object.entries(system).filter(([key]) => key !== "Qcoordinates" && key !== "initconditions")
                .map(([key, value]) => [key, evaluateMatrix(value, state)])
        )
    };
    console.timeEnd("Matrix Evaluation");

    console.log("Evaluated Matrices:", evaluatedMatrices);

    return { state, evaluateExpression, evaluateMatrix, evaluatedMatrices, system };
}

export function evaluateExpression(entry, state) {
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

    const missingVars = variables.filter(v => !(v in state));
    if (missingVars.length > 0) {
        console.error("Missing variables in state:", missingVars);
        return NaN;
    }

    try {
        return new Function(...variables, `return ${expr};`)(...variables.map(v => state[v]));
    } catch (e) {
        console.error("Eval failed for:", expr, "Error:", e);
        return NaN;
    }
}

export function evaluateMatrix(matrix, state) {
    if (!Array.isArray(matrix)) return evaluateExpression(matrix, state);
    return matrix.map(row => Array.isArray(row) ? row.map(entry => evaluateExpression(entry, state)) : evaluateExpression(row, state));
}
