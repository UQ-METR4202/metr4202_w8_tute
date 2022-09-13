import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['text.usetex'] = True

m1 = 1
m2 = 1
L1 = 1
I1 = 1
I2 = 1
g0 = 9.81

def get_K(q1, q2, q1dot, q2dot):
    return 0.5 * m2 * q2dot ** 2 + 0.5 * (m1 * L1 ** 2 + m2 * q2 ** 2 + I1 + I2) * q1dot ** 2

def get_P(q1, q2):
    return (m1 * L1 + m2 * q2) * g0 * np.sin(q1)

def get_M(q1, q2):
    return np.array([
        [m1 * L1 ** 2 + m2 * q2 ** 2 + I1 + I2, 0],
        [0, m2]
    ])

def get_C(q1, q2, q1dot, q2dot):
    return m2 * q2 * q2dot * np.diag([2, -1])

def get_g(q1, q2):
    return np.array(
        [(m1 * L1 + m2 * q2) * g0 * np.cos(q1), m2 * g0 * np.sin(q1)]
    )

if __name__ == "__main__":
    N = 1000000
    q = np.zeros((2, N + 1))
    qdot = np.zeros((2, N + 1))
    q[0, 0] = np.pi / 4
    dt = 1e-6
    for n in range(N):
        M = get_M(q[0, n], q[1, n])
        C = get_C(q[0, n], q[1, n], qdot[0, n], qdot[1, n])
        g = get_g(q[0, n], q[1, n])
        
        qddot = np.linalg.inv(M) @ ( - C @ qdot[:, n] - g)
        qdot[:, n + 1]  = qdot[:, n]    + qddot * dt
        q[:, n + 1]     = q[:, n]       + qdot[:, n] * dt

        K = get_K(q[0, n], q[1, n], qdot[0, n], qdot[1, n])
        P = get_P(q[0, n], q[1, n])
        print(K + P)

    plt.plot(q[0, :])
    plt.plot(q[1, :])
    plt.legend([r'$\theta_{1}$', r'$\theta_{2}$'])
    plt.show()
    