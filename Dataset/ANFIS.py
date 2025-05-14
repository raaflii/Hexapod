import numpy as np
import skfuzzy as fuzz
import pandas as pd
import matplotlib.pyplot as plt

# Load dataset
data = pd.read_csv('L1.csv') 

x = data['roll'].values
y = data['pitch'].values
z = data['koreksi_z'].values

x_neg = fuzz.trimf(x, [-30, -15, 0])  
x_pos = fuzz.trimf(x, [0, 15, 30])    

y_neg = fuzz.trimf(y, [-30, -15, 0])  
y_pos = fuzz.trimf(y, [0, 15, 30])    

# RULE FIRING STRENGTH
w1 = x_neg * y_neg 
w2 = x_neg * y_pos  
w3 = x_pos * y_neg  
w4 = x_pos * y_pos  

# NORMALISASI
wsum = w1 + w2 + w3 + w4
wsum_safe = np.where(wsum == 0, 1e-6, wsum)

w1n = w1 / wsum_safe
w2n = w2 / wsum_safe
w3n = w3 / wsum_safe
w4n = w4 / wsum_safe

def train_rule(w, x, y, z):
    X = np.vstack((w * x, w * y, w)).T
    y_target = w * z
    theta, _, _, _ = np.linalg.lstsq(X, y_target, rcond=None)
    return theta  # p, q, r

p1, q1, r1 = train_rule(w1n, x, y, z)
p2, q2, r2 = train_rule(w2n, x, y, z)
p3, q3, r3 = train_rule(w3n, x, y, z)
p4, q4, r4 = train_rule(w4n, x, y, z)

f1 = p1 * x + q1 * y + r1
f2 = p2 * x + q2 * y + r2
f3 = p3 * x + q3 * y + r3
f4 = p4 * x + q4 * y + r4

output = w1n * f1 + w2n * f2 + w3n * f3 + w4n * f4

mse = np.mean((z - output) ** 2)

print("===== PARAMETER ANFIS 4 RULE (untuk satu kaki) =====")
for i, (p, q, r) in enumerate([(p1,q1,r1), (p2,q2,r2), (p3,q3,r3), (p4,q4,r4)], start=1):
    print(f"Rule {i}: f{i} = {p:.4f} * x + {q:.4f} * y + {r:.4f}")

print(f"\nMSE: {mse:.4f}")

# VISUALISASI
plt.figure(figsize=(10, 5))
plt.plot(z, label='Target koreksi Z', marker='o')
plt.plot(output, label='ANFIS Output', linestyle='--', marker='x')
plt.title('Perbandingan Output ANFIS vs Target (4 Rule)')
plt.xlabel('Data ke-')
plt.ylabel('Koreksi Z')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
