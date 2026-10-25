import math
import matplotlib.pyplot as plt

N_MAX = 2000
TARGET_N = 100
TARGET_SCORE = 0.5

base_at_target = math.log1p(TARGET_N) / math.log1p(N_MAX)
gamma = math.log(TARGET_SCORE) / math.log(base_at_target)

def score(n: int) -> float:
    if n <= 0:
        return 0.0
    base = math.log1p(n) / math.log1p(N_MAX)
    return min(base ** gamma, 1.0)

x = list(range(1, N_MAX + 1))
y = [score(n) for n in x]

print(f"gamma = {gamma:.6f}")
for n in [1, 10, 50, 100, 150, 200, 500, 1000, 1500, 2000]:
    print(f"{n:4d} -> {score(n):.4f}")

plt.figure(figsize=(10, 6))
plt.plot(x, y, linewidth=2)
plt.axhline(0.5, linestyle='--')
plt.axvline(200, linestyle='--')
plt.xlabel("Frontier grid count N")
plt.ylabel("Score")
plt.title("Log-normalized score with score(200)=0.5")
plt.grid(True)
plt.tight_layout()
plt.show()