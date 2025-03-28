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

    // Store trig functions for evaluation
    let trigMap = {};
    system.trigFunctions.forEach(trig => {
        trigMap[trig.name] = trig.expr;
    });

    console.log("Initial Q", system.initconditions);
    let Q = system.initconditions.map(row => [...row]);

    return { system, Q, variableMap, trigMap };
}

const compiledExpressions = new Map(); // Cache compiled expressions

export function computeTrigValues(Q, variableMap, trigMap) {
    let trigValues = {};
    Object.entries(trigMap).forEach(([trigName, expr]) => {
        try {
            const fn = new Function(...Object.keys(variableMap), `return ${expr};`);
            const values = Object.keys(variableMap).map(v => Q[variableMap[v].i][variableMap[v].j]);
            trigValues[trigName] = fn(...values);
        } catch (e) {
            console.error(`Failed to compute ${trigName}: ${expr}`, e);
            trigValues[trigName] = NaN;
        }
    });
    return trigValues;
}

export function evaluateExpression(entry, Q, variableMap, trigValues) {
    if (!entry || typeof entry.expr !== "string") {
        console.warn("Invalid entry:", entry);
        return NaN;
    }

    const { expr, vars } = entry;
    const variables = Array.isArray(vars) ? vars : vars?.trim() ? [vars] : [];

    if (variables.length === 0) {
        if (!compiledExpressions.has(expr)) {
            try {
                compiledExpressions.set(expr, new Function(`return ${expr};`)());
            } catch (e) {
                console.error("Eval failed for constant expression:", expr, "Error:", e);
                return NaN;
            }
        }
        return compiledExpressions.get(expr);
    }

    try {
        if (!compiledExpressions.has(expr)) {
            const fn = new Function(...variables, `return ${expr};`);
            compiledExpressions.set(expr, fn);
        }

        const fn = compiledExpressions.get(expr);
        const values = variables.map(v => {
            return v.startsWith("trig") ? trigValues[v] : Q[variableMap[v].i][variableMap[v].j];
        });

        return fn(...values);
    } catch (e) {
        console.error("Eval failed for:", expr, "Error:", e);
        return NaN;
    }
}

export async function evaluateMatrix(matrix, Q, variableMap, trigValues) {
    if (!Array.isArray(matrix)) return evaluateExpression(matrix, Q, variableMap, trigValues);
    return Promise.all(matrix.map(row =>
        Array.isArray(row) ? Promise.all(row.map(entry => evaluateExpression(entry, Q, variableMap, trigValues))) : evaluateExpression(row, Q, variableMap, trigValues)
    ));
}
