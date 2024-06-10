import csv
from ortools.linear_solver import pywraplp

# Define the input data
data = [
    {'Diameter': 12, 'Length': 12, 'Number': 3976},
    {'Diameter': 12, 'Length': 5.405, 'Number': 142},
    {'Diameter': 12, 'Length': 6, 'Number': 142},
    {'Diameter': 12, 'Length': 6.82, 'Number': 142},
    {'Diameter': 12, 'Length': 7.26, 'Number': 142},
    {'Diameter': 12, 'Length': 8.245, 'Number': 142},
    {'Diameter': 12, 'Length': 9.49, 'Number': 142},
    {'Diameter': 12, 'Length': 9.66, 'Number': 142},
    {'Diameter': 12, 'Length': 11.08, 'Number': 142},
    {'Diameter': 12, 'Length': 11.265, 'Number': 142},
    {'Diameter': 12, 'Length': 11.68, 'Number': 142},
    {'Diameter': 12, 'Length': 11.735, 'Number': 142},
    {'Diameter': 12, 'Length': 11.79, 'Number': 142},
    {'Diameter': 12, 'Length': 11.845, 'Number': 142},
    {'Diameter': 12, 'Length': 11.895, 'Number': 142},
    {'Diameter': 12, 'Length': 11.95, 'Number': 142},
    {'Diameter': 12, 'Length': 5.955, 'Number': 426},
    {'Diameter': 12, 'Length': 7.575, 'Number': 426},
    {'Diameter': 16, 'Length': 3.885, 'Number': 748},
    {'Diameter': 12, 'Length': 6.205, 'Number': 426},
    {'Diameter': 12, 'Length': 7.57, 'Number': 426},
    {'Diameter': 12, 'Length': 8.935, 'Number': 426},
    {'Diameter': 12, 'Length': 9.22, 'Number': 1278},
    {'Diameter': 12, 'Length': 8.87, 'Number': 576},
    {'Diameter': 12, 'Length': 8.77, 'Number': 1128},
    {'Diameter': 12, 'Length': 7.14, 'Number': 376},
    {'Diameter': 12, 'Length': 7.475, 'Number': 376},
    {'Diameter': 12, 'Length': 7.935, 'Number': 376},
    {'Diameter': 12, 'Length': 8.27, 'Number': 376},
    {'Diameter': 12, 'Length': 8.615, 'Number': 376},
    {'Diameter': 12, 'Length': 7.22, 'Number': 192},
    {'Diameter': 12, 'Length': 7.54, 'Number': 192},
    {'Diameter': 12, 'Length': 7.97, 'Number': 192},
    {'Diameter': 12, 'Length': 8.29, 'Number': 192},
    {'Diameter': 12, 'Length': 8.73, 'Number': 192},
   ]
with open('../LnT Kalyani Input.csv') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        data.append({'diameter': int(row['Diameter']), 'length': float(row['Length']), 'number': int(row['Number'])})

# Define the rebar length and the number of pieces required
rebar_length = 12
num_pieces = sum(d['number'] for d in data)

# Define the solver
solver = pywraplp.Solver('CAB Rebar Optimization Model', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

# Define the variables
cut_vars = {}
for i, d in enumerate(data):
    cut_vars[i] = solver.IntVar(0, d['number'], 'CutVar_%d' % i)

# Define the objective function
solver.Maximize(solver.Sum([cut_vars[i] for i in cut_vars]))

# Define the constraints
for i, d in enumerate(data):
    solver.Add(cut_vars[i] * d['length'] <= rebar_length)

# Define the column generation function
def generate_column():
    # Generate a new cutting pattern
    pattern = []
    for d in data:
        if d['length'] < rebar_length and d['number'] > 0:
            num_pieces_to_cut = min(d['number'], int(rebar_length / d['length']))
            pattern.append((d['length'] * num_pieces_to_cut, d['length'], num_pieces_to_cut))
            d['number'] -= num_pieces_to_cut
    return pattern

# Define the column generation loop
while True:
    # Solve the current LP relaxation
    solver.Solve()

    # Check if the solution is integer
    if solver.Integer():
        break

    # Generate a new cutting pattern
    new_pattern = generate_column()

    # Add the new cutting pattern to the model
    if new_pattern:
        new_vars = []
        for j in range(len(new_pattern)):
            length, pattern_length, num_pieces = new_pattern[j]
            new_var = solver.IntVar(0, num_pieces, 'NewVar_%d' % j)
            new_vars.append(new_var)
            solver.Add(new_var == num_pieces)
            solver.Add(new_var * pattern_length <= rebar_length)
            solver.Add(new_var * pattern_length >= rebar_length * (1 - (1 - pattern_length) / rebar_length))
            solver.Add(new_var >= 0)
            solver.Add(cut_vars[data.index({'diameter': length, 'length': pattern_length, 'number': 1})] >= new_var)

# Print the results
print('Number of pieces required:', num_pieces)
print('Number of cuts:', solver.Objective().Value())
print('Yield:', solver.Objective().Value() / num_pieces)

# Plot the new cutting patterns
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(10, 6))
for i, d in enumerate(data):
    ax.barh(y=i, width=d['length'] * cut_vars[i].solution_value(), height=0.5, left=0, label=f'Diameter {d["diameter"]}')
ax.set_xlabel('Length (m)')
ax.set_ylabel('Rebar')
ax.set_title('Cut and Bent (CAB) Rebar Optimization Model')
ax.leg