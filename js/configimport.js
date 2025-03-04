// Load the system config JSON file
fetch('../src/FrameGen/systemconfig.json')
  .then(response => response.json())
  .then(system => {
      const h = 0.01; // Time step
      let state = { x: 1.5, y: 2.0, z: Math.PI / 4 }; // Initial conditions

      function evaluateExpression(expr, vars) {
          let result = expr;
          vars.forEach(varName => {
              result = result.replace(new RegExp(`\\b${varName}\\b`, 'g'), state[varName]);
          });
          return eval(result);
      }

      function evaluateMatrix(matrix) {
          return matrix.map(row =>
              row.map(entry => evaluateExpression(entry.expr, entry.vars))
          );
      }

      // Process all matrices dynamically
      let evaluatedMatrices = {};
      Object.keys(system).forEach(key => {
          if (key !== "variables") {
              evaluatedMatrices[key] = evaluateMatrix(system[key]);
          }
      });

    console.log("Initial Q:", Q);
    return { system, Q, variableMap };
}

const compiledExpressions = new Map(); // Cache compiled expressions

export function evaluateExpression(entry, Q, variableMap) {
    if (!entry || typeof entry.expr !== "string") {
        console.warn("Invalid entry:", entry);
        return NaN;
    }

    const { expr, vars } = entry;
    const variables = Array.isArray(vars) ? vars : vars?.trim() ? [vars] : [];

    if (variables.length === 0) {
        // Cache constant expressions
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

    const missingVars = variables.filter(v => !(v in variableMap));
    if (missingVars.length > 0) {
        console.error("Missing variables in Q:", missingVars);
        return NaN;
    }

    try {
        // Precompile expressions if not already cached
        if (!compiledExpressions.has(expr)) {
            const fn = new Function(...variables, `return ${expr};`);
            compiledExpressions.set(expr, fn);
        }

        const fn = compiledExpressions.get(expr);
        const values = variables.map(v => {
            let { i, j } = variableMap[v];
            return Q[i][j];
        });

        return fn(...values);
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


