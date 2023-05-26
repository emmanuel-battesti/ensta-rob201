"""Simple demonstration of numpy speedup"""
import timeit
import numpy as np

ranges = np.random.rand(360)
ray_angles = np.arange(-np.pi, np.pi, np.pi / 180)


def pol_to_cart1():
    """Poor implementation of polar to cartesian conversion"""
    points = []
    for i in range(360):
        pt_x = ranges[i] * np.cos(ray_angles[i])
        pt_y = ranges[i] * np.sin(ray_angles[i])
        points.append([pt_x, pt_y])
    return np.array(points).T


def pol_to_cart2():
    """Better implementation of polar to cartesian conversion"""
    pts_x = ranges * np.cos(ray_angles)
    pts_y = ranges * np.sin(ray_angles)
    return np.vstack((pts_x, pts_y))


total_time1 = timeit.timeit("pol_to_cart1()",
                            "from __main__ import pol_to_cart1, ranges, ray_angles",
                            number=1000)
print(f"pol_to_cart1 : {total_time1:.3f} s")

total_time2 = timeit.timeit("pol_to_cart2()",
                            "from __main__ import pol_to_cart2, ranges, ray_angles",
                            number=1000)
print(f"pol_to_cart2 : {total_time2:.3f} ms")

print(f"speedup : x {total_time1 / total_time2:.1f}")
