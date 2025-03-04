import sympy as sp
import time
import json

# Load JSON file
with open('CraneConfig.json', 'r') as file:
    config = json.load(file)

# Extract symbolic expressions from Nstar
expressions = []
variables = set()
for row in config["B"]:
    expr_row = []
    for entry in row:
        expr_str = entry["expr"].replace("Math.", "")  # Adjust JavaScript-style math
        expr_sympy = sp.sympify(expr_str)
        expr_row.append(expr_sympy)
        variables.update(entry["vars"])
    expressions.append(expr_row)

# Define symbolic variables
symbols = {var: sp.Symbol(var) for var in variables}

# Convert matrix to SymPy
test_matrix = sp.Matrix(expressions)

# Measure simplification time
start_time = time.time()
simplified_matrix = test_matrix.applyfunc(sp.simplify)
end_time = time.time()

# Choose an element to compare (row 0, col 0 for example)
chosen_element = simplified_matrix[48-1-2, 1-1]


print("\nSimplified matrix:")
sp.pprint(simplified_matrix)
print(f"\nSimplification time: {end_time - start_time:.4f} seconds")

# Print chosen element for comparison
print("\nChosen simplified element (0,0):")
sp.pprint(chosen_element)
