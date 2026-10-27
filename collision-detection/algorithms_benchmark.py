"""
Computing time comparison of Brute Force / Sweep and Prune / BVH.
"""
import random, time
import matplotlib
import matplotlib.pyplot as plt
from collision_algorithms import ALGORITHMS, build_bvh, bvh_query

WORLD = 50.0          # world is a cube [-25,25]^3
SIZES = [100, 300, 700, 1500, 3000, 6000, 10000]
REPEATS = 3

# Generates boxes
def random_boxes(n, dim=3, seed=0):
    rnd = random.Random(seed)
    out = []
    for _ in range(n):
        c = [rnd.uniform(-WORLD / 2, WORLD / 2) for _ in range(3)]      # center of the box
        h = [rnd.uniform(0.25, 1.5) for _ in range(3)]             # half measure of the box
        if dim == 2:
            c[2], h[2] = 0.0, 0.0                                       # 2D ==> z = 0
        out.append((c[0]-h[0], c[1]-h[1], c[2]-h[2], c[0]+h[0], c[1]+h[1], c[2]+h[2]))  # bounding box for object
    return out

# Calculates best time for an algorithm
def best_time(fn, *args):
    best, res = float("inf"), None
    for _ in range(REPEATS):
        t = time.perf_counter()
        res = fn(*args)
        best = min(best, time.perf_counter() - t)
    return best * 1000, res  # result in ms


def run(dim):
    print(f"\n=== BENCHMARK {dim}D ===")
    header = f"{'N':>6} | " + " | ".join(f"{k:>15}" for k in ALGORITHMS) + " |    BVH Query | Collisions"
    print(header)
    print("-" * len(header))

    results = {k: [] for k in ALGORITHMS}
    results["BVH (Query Only)"] = []

    for n in SIZES:
        boxes = random_boxes(n, dim)    # random AABB
        row, ref = [], None             # list for times and expected result

        # Standard tests
        for name, fn in ALGORITHMS.items():
            ms, (pairs, checks) = best_time(fn, boxes)
            if ref is None:
                ref = pairs
            assert pairs == ref, f"ERROR: {name} gives different result than Brute Force!"
            results[name].append(ms)
            row.append(f"{ms:12.2f} ms")

        # Test for BVH Query
        tb, (nodes, root) = best_time(build_bvh, boxes)
        tq, (pairs_q, _) = best_time(bvh_query, boxes, nodes, root)
        results["BVH (Query Only)"].append(tq)

        print(f"{n:>6} | " + " | ".join(f"{r:>15}" for r in row) + f" | {tq:9.2f} ms | {len(ref):>10}")

    return results


if __name__ == "__main__":

    matplotlib.use("Agg") # renders .png instead of showing in IDE
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for ax, dim in zip(axes, (2, 3)):
        res = run(dim)
        for name, ts in res.items():
            style = "--" if "Query Only" in name else "-"
            linewidth = 2.5 if "BVH" in name else 1.5
            ax.plot(SIZES, ts, marker="o", linestyle=style, linewidth=linewidth, label=name)

        ax.set_title(f"Collision detection in {dim}D")
        ax.set_xlabel("Number of objects (N)")
        ax.set_ylabel("Computing time [ms] (logarithmic scale)")
        ax.set_yscale("log")
        ax.set_xscale("log")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()

    plt.tight_layout()
    plt.savefig("benchmark.png", dpi=150)
    print("\nPlot was successfully generated and saved in 'benchmark.png'")