import sys, numpy as np, matplotlib
if '--show' not in sys.argv: matplotlib.use('Agg')
import matplotlib.pyplot as plt
o = np.loadtxt('odom_raw.txt'); ot = o[:, 0] / 1e9; oxy = o[:, 1:3]

def umeyama(A, B):  # rigid A->B
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((A - ma).T @ (B - mb))
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T)); R = Vt.T @ D @ U.T
    return R, mb - R @ ma

fig, ax = plt.subplots(1, 2, figsize=(13, 6))
for k, f in enumerate(('odom_cuvslam.tum', 'slam_cuvslam.tum')):
    a = np.loadtxt(f); t = a[:, 0]; p = a[:, 1:4]
    ax[k].plot(oxy[:, 0], oxy[:, 1], 'k-', lw=2, label='K1 leg odometer')
    sl = slice(866, None); P = p[sl]; T = t[sl]
    G = np.c_[np.interp(T, ot, oxy[:, 0]), np.interp(T, ot, oxy[:, 1]), np.zeros(len(T))]
    R, tr = umeyama(P, G); Q = (R @ P.T).T + tr
    e = np.sqrt((np.linalg.norm(Q[:, :2] - G[:, :2], axis=1) ** 2).mean())
    ax[k].plot(Q[:, 0], Q[:, 1], '-', c='tab:orange', label=f'cuVSLAM after stall (ATE {e:.2f} m)')
    ax[k].plot(*G[0, :2], 'go', ms=9, label='cam resumes'); ax[k].plot(*oxy[0], 'bs', ms=9, label='start')
    ax[k].set_title(('VO' if k == 0 else 'SLAM') + ' vs leg odometer (rigid-aligned)')
    ax[k].axis('equal'); ax[k].grid(); ax[k].legend(fontsize=8); ax[k].set_xlabel('x [m]'); ax[k].set_ylabel('y [m]')
plt.tight_layout(); plt.savefig('traj.png', dpi=80)
if '--show' in sys.argv: plt.show()
