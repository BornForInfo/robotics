import numpy as np
import matplotlib.pyplot as plt

def array():
    rng = np.random.default_rng(seed=42)
    array_a = rng.choice([-1, 1], size = 1024)
    array_b = array_a[256:767]
    array_c = []
    for t in range(len(array_b)):
        result = 0
        for i in range(len(array_b)):
            result += array_a[i+t]*array_b[i]
        array_c.append(result)
    x = list(range(len(array_c)))

    return x, array_c

def noise(array_c, prob):
    rng = np.random.default_rng()
    flip_mask = rng.random(size=len(array_c)) < prob
    array_c = np.array(array_c)
    array_c[flip_mask] *= -1
    return array_c

x, array_c = array()
a_flip_array_c = noise(array_c, 0.05)
b_flip_array_c = noise(array_c, 0.1)
c_flip_array_c = noise(array_c, 0.2)
d_flip_array_c = noise(array_c, 0.5)

fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize = (10, 4))

ax1.plot(x, array_c)
ax1.set_title('raw')
ax2.plot(x, a_flip_array_c)
ax2.set_title('prob_0.05')
ax3.plot(x, b_flip_array_c)
ax3.set_title('prob_0.1')
ax4.plot(x, c_flip_array_c)
ax4.set_title('prob_0.2')
ax5.plot(x, d_flip_array_c)
ax5.set_title('prob_0.5')
plt.show()
