# 🤖 Line Following + Lidar Mapping + Homogeneous Transform (Webots)

## 📌 Overview

This project extends a basic **line-following robot** into a more advanced robotic system capable of:

- Estimating its own position (odometry)
- Perceiving the environment using a **360° Lidar**
- Transforming sensor data into the **world coordinate system**
- Visualizing mapped obstacles in real time

The project is structured progressively, starting from a simple controller and evolving into a more advanced implementation using **homogeneous transformations**.

---

## 🧭 1. Initial Controller: Line Following

The base controller implements a simple **line-following behavior** using ground sensors:

```python
if (g[0]>0 and g[1] < 250 and g[2] > 500):
    phildot, phirdot = 0.3 * MAX_SPEED, 0.3 * MAX_SPEED
elif (g[2] < 550):
    phildot, phirdot = 0.1 * MAX_SPEED, -0.1 * MAX_SPEED
elif (g[1] > 500):
    phildot, phirdot = -0.1 * MAX_SPEED, 0.1 * MAX_SPEED
```

The robot adjusts wheel velocities (while turning) to stay on the line.

---

## 📍 2. Odometry Implementation

To estimate the robot's position, we compute:

* Linear displacement
* Angular rotation

```python
v_l = WHEEL_RADIUS * phildot
v_r = WHEEL_RADIUS * phirdot

delta_x = (v_l + v_r) / 2 * dt
delta_wz = (v_r - v_l) / WHEEL_DISTANCE * dt
```

Update robot pose:

```python
omegaz += delta_wz
xw += np.cos(omegaz) * delta_x
yw += np.sin(omegaz) * delta_x
```

This provides an approximation of the robot's position in the world frame.

---

## 📡 3. Adding the Lidar (Range Finder)

A 360° Lidar (`LDS-01`) is added:

```python
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()
```

We retrieve distance measurements:

```python
ranges = lidar.getRangeImage()
```

Each measurement corresponds to an angle:

```python
angles = np.linspace(3.1415, -3.1415, 360)
```

---

## 🔄 4. Step 1 — Lidar to Robot Coordinates

Each Lidar point is converted from polar to Cartesian coordinates:

```python
x_i = range * np.cos(angle)
y_i = range * np.sin(angle)
```

These points are expressed in the **robot frame**.

---

## 🌍 5. Step 2 — Robot to World Transformation (Rotation + Translation)

We transform points into the world frame using a rotation matrix:

[
R(\theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta \
\sin\theta & \cos\theta
\end{bmatrix}
]

Implementation:

```python
w_R_r = np.array([
    [np.cos(omegaz), -np.sin(omegaz)],
    [np.sin(omegaz),  np.cos(omegaz)]
])

X_w = np.array([[xw], [yw]])
Data = w_R_r @ np.array([[x_i], [y_i]]) + X_w
```

This step converts Lidar points from the robot frame to the **world frame**.

---

## ⚡ 6. Limitations of the First Approach

* Uses a loop (`for`)
* Processes each point individually
* Less efficient for large datasets

---

## 🚀 7. Step 3 — Homogeneous Transformation

To improve efficiency and clarity, we use a homogeneous transformation matrix:

[
T =
\begin{bmatrix}
\cos\theta & -\sin\theta & x_w \
\sin\theta &  \cos\theta & y_w \
0 & 0 & 1
\end{bmatrix}
]

---

## 🧮 Vectorized Implementation

All Lidar points are processed at once:

```python
ranges = np.array(lidar.getRangeImage())
ranges[ranges == np.inf] = 0

X_i = np.array([
    ranges * np.cos(angles),
    ranges * np.sin(angles),
    np.ones_like(ranges)
])

Data = w_T_r @ X_i
```

Where:

```python
w_T_r = np.array([
    [np.cos(omegaz), -np.sin(omegaz), xw],
    [np.sin(omegaz),  np.cos(omegaz), yw],
    [0, 0, 1]
])
```

---

## 📊 8. Visualization

Mapped points are displayed using Matplotlib:

```python
plt.plot(Data[0,:], Data[1,:], '.')
plt.pause(0.01)
```

This provides a real-time visualization of the environment.

---

## 🔁 9. Final Behavior

The robot simultaneously:

* Follows a line
* Estimates its pose (x, y, orientation)
* Maps surrounding obstacles using Lidar
* Projects them into the world frame

---

## 🧠 Key Concepts Covered

* Differential drive kinematics
* Odometry
* Coordinate transformations
* Rotation matrices
* Homogeneous transformations
* Lidar data processing
* Real-time mapping

---

## 📌 Notes

* Odometry accumulates error over time
* Lidar data may include noise (`inf` values filtered)
* Homogeneous transformation improves performance and readability

---

## 🎯 Conclusion

This project demonstrates the transition from:

➡️ Basic reactive control (line following)
➡️ To perception-driven robotics with spatial awareness

It forms a foundation for more advanced topics such as:

* SLAM (Simultaneous Localization and Mapping)
* Autonomous navigation
* Sensor fusion


