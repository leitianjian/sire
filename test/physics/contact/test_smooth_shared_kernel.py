"""Analytic contact-law and unilateral-normal tests for the shared kernel."""
import json
import numpy as np
import sire


def solve(G, velocity, target, mu, vs, h=.02, acceleration=None):
    n = len(mu)
    acceleration = np.zeros(3*n) if acceleration is None else np.asarray(acceleration)
    report = sire.cptContactForceSmoothLinear(
        list(mu), (-G).ravel().tolist(), list(velocity), list(target),
        acceleration.tolist(), h, list(vs))
    f = np.asarray(report["force"])
    c = np.asarray(velocity) - h * acceleration + h * G @ f
    gap = c[2::3] + np.asarray(target)
    assert np.min(f[2::3]) >= -1e-9
    assert np.min(gap) >= -1e-8
    assert np.max(np.abs(gap * h * f[2::3])) < 1e-7
    for i in range(n):
        v = np.asarray(velocity[3*i:3*i+2])
        expected = -mu[i] * v / max(np.linalg.norm(v), vs[i]) * f[3*i+2]
        np.testing.assert_allclose(f[3*i:3*i+2], expected, rtol=1e-10, atol=1e-9)
        assert np.dot(f[3*i:3*i+2], v) <= 1e-9
    return f, report


checks = []
for speed in (0.0, .05, .3, .6):
    f, _ = solve(np.eye(3), [speed, 0, -.2], [0], [.4], [.3])
    np.testing.assert_allclose(f[2], 10.0, atol=1e-9)
    np.testing.assert_allclose(f[0], -4.0 * min(speed / .3, 1), atol=1e-9)
checks.append("zero velocity, linear region, threshold and saturation")
f, _ = solve(np.eye(3), [.1, .2, .2], [0], [.4], [.3])
np.testing.assert_allclose(f, 0, atol=1e-12)
checks.append("separation without attractive force")
f, _ = solve(np.eye(3), [.1, .2, -.2], [.05], [0], [.3])
np.testing.assert_allclose(f, [0, 0, 7.5], atol=1e-9)
checks.append("normal DAE target and frictionless limit")
for h in (1e-6, .001, .02):
    f, _ = solve(np.eye(3), [.05, 0, -.2], [0], [.4], [.3], h=h)
    np.testing.assert_allclose(h*f[2], .2, atol=1e-10)
checks.append("timestep-independent impulse solve")
G = np.eye(6)
G[2, 5] = G[5, 2] = 1.0
f, result = solve(G, [0, 0, -.3, 0, 0, -.2], [0, 0], [.4, .4], [.3, .3])
assert result["enumerated"]
np.testing.assert_allclose(f[[2, 5]], [15, 0], atol=1e-8)
checks.append("degenerate shared normal direction with active-set verification")
rng = np.random.default_rng(2)
for _ in range(50):
    n = 4
    A = rng.normal(size=(3*n, 3*n))
    G = np.eye(3*n) + .02 * A @ A.T
    velocity = rng.normal(size=3*n)
    solve(G, velocity, np.zeros(n), np.full(n, .4), np.full(n, .3))
checks.append("coupled contact response with 50 independent fixtures")
print(json.dumps({"ready": True, "checks": checks}, indent=2))
