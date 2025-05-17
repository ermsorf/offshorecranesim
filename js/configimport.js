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
    console.log("System Configuration:", system)
    console.log("Initial Q", system.initconditions);
    let Q = system.initconditions.map(row => [...row]);
    // Q[4][0] = -10;
    Q[2][0] = Math.PI/2

    return { system, Q, variableMap, trigMap };
}


const compiledExpressions = new Map(); // Cache compiled expressions

export function computeTrigValues(Q, variableMap, trigMap, constants = {}) {
    const trigValues = {};
    Object.entries(trigMap).forEach(([trigName, expr]) => {
        try {
            let processedExpr = expr;
            Object.entries(variableMap).forEach(([varName, { i, j }]) => {
                processedExpr = processedExpr.replaceAll(varName, Q[i][j]);
            });
            Object.entries(constants).forEach(([constName, value]) => {
                processedExpr = processedExpr.replaceAll(constName, value);
            });

            const fn = new Function(`return ${processedExpr};`);
            trigValues[trigName] = fn();
        } catch (e) {
            console.error(`Failed to compute ${trigName}: ${expr}`, e);
            trigValues[trigName] = NaN;
        }
    });
    return trigValues;
}

export function evaluateExpression(entry, Q, variableMap, trigValues = {}, constants = {}) {
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
            if (v in variableMap) {
                const { i, j } = variableMap[v];
                return Q[i][j];
            } else if (v in trigValues) {
                return trigValues[v];
            } else if (v in constants) {
                return constants[v];
            } else {
                throw new Error(`Missing variable '${v}' in Q, trigValues, and constants`);
            }
        });

        return fn(...values);
    } catch (e) {
        console.error("Eval failed for:", expr, "Error:", e);
        return NaN;
    }
}

export async function evaluateMatrix(matrix, Q, variableMap, trigValues = {}, constants = {}) {
    if (!Array.isArray(matrix)) {
        return evaluateExpression(matrix, Q, variableMap, trigValues, constants);
    }

    return Promise.all(
        matrix.map(row =>
            Array.isArray(row)
                ? Promise.all(row.map(entry => evaluateExpression(entry, Q, variableMap, trigValues, constants)))
                : evaluateExpression(row, Q, variableMap, trigValues, constants)
        )
    );
}
