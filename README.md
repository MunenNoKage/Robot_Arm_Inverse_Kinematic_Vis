# Robot Arm Simulation with Inverse Kinematics and 3D Projection

> A Linear Algebra course project demonstrating how matrix transformations, rotation groups, and projection geometry power real-time robotic simulation.

---

## Project Overview

This project implements a **3D robot arm simulator** that combines two core computational challenges:

1. **Inverse Kinematics (IK)** — given a desired end-effector (gripper) position in space, compute the joint angles needed to reach it.
2. **3D-to-2D Projection** — render the arm in real time on a 2D screen using perspective or orthographic projection matrices.

The entire system is built on **linear algebra primitives**: rotation matrices, homogeneous coordinates, Jacobian matrices, and projection transforms. Every frame rendered and every joint angle solved is a direct application of concepts from a Linear Algebra course.

---

## Project Goals

| Goal | Description |
|------|-------------|
| **Mathematical modeling** | Represent each robot joint as a linear transformation in 3D space |
| **Forward Kinematics** | Compute end-effector position from joint angles using matrix chains |
| **Inverse Kinematics** | Solve for joint angles given a target position (Jacobian-based method) |
| **3D Rendering** | Project the 3D skeleton onto a 2D viewport with perspective projection |
| **Real-time visualization** | Animate smooth motion as the arm reaches toward moving targets |
| **Linear algebra showcase** | Every operation traceable to a matrix multiplication or decomposition |

---

## Mathematical Foundation

### 1. Homogeneous Coordinates

To unify rotation and translation into a single matrix operation, we use **homogeneous coordinates** — representing a 3D point `(x, y, z)` as a 4D vector:

```
p = [x, y, z, 1]ᵀ
```

A transformation (rotate + translate) becomes a single 4×4 matrix multiply:

```
p' = T · R · p
```

where `T` is a translation matrix and `R` is a rotation matrix embedded in 4×4 form.

---

### 2. Rotation Matrices (SO(3))

Rotation around each principal axis is represented by a matrix in the **special orthogonal group SO(3)**:

**Rotation around X-axis by angle θ:**
```
Rₓ(θ) = | 1    0       0    |
         | 0   cos θ  -sin θ |
         | 0   sin θ   cos θ |
```

**Rotation around Y-axis:**
```
R_y(θ) = |  cos θ  0  sin θ |
         |    0    1    0   |
         | -sin θ  0  cos θ |
```

**Rotation around Z-axis:**
```
R_z(θ) = | cos θ  -sin θ  0 |
         | sin θ   cos θ  0 |
         |   0       0    1 |
```

Key properties used throughout:
- `R⁻¹ = Rᵀ` (rotation matrices are orthogonal)
- `det(R) = 1`
- Composing rotations = multiplying matrices (non-commutative!)

---

### 3. Forward Kinematics (FK)

For a robot arm with `n` joints, each joint `i` has:
- A joint angle `θᵢ`
- A local rotation matrix `Rᵢ(θᵢ)`
- A translation (bone length) `tᵢ`

The **transformation matrix** for joint `i` in homogeneous form:

```
Tᵢ(θᵢ) = | Rᵢ(θᵢ)  tᵢ |
           |   0     1  |
```

The **end-effector position** (tip of the arm) is obtained by chaining all joint transforms:

```
T_end = T₁(θ₁) · T₂(θ₂) · T₃(θ₃) · ... · Tₙ(θₙ)
```

The final position in world space:

```
p_end = T_end · [0, 0, 0, 1]ᵀ
```

This is pure matrix multiplication — the entire kinematics chain is a product of 4×4 matrices.

---

### 4. Inverse Kinematics via the Jacobian

**Inverse Kinematics** asks: given a desired position `p_target`, find `θ = [θ₁, θ₂, ..., θₙ]` such that `FK(θ) = p_target`.

This is solved iteratively using the **Jacobian matrix** `J`, which relates small changes in joint angles to small changes in end-effector position:

```
Δp ≈ J(θ) · Δθ
```

where `J` is a `3×n` matrix (3 spatial dimensions, n joints):

```
J = | ∂x/∂θ₁  ∂x/∂θ₂  ...  ∂x/∂θₙ |
    | ∂y/∂θ₁  ∂y/∂θ₂  ...  ∂y/∂θₙ |
    | ∂z/∂θ₁  ∂z/∂θ₂  ...  ∂z/∂θₙ |
```

Each column `j` of the Jacobian for a revolute joint is:

```
Jⱼ = axis_j × (p_end - p_j)
```

where `axis_j` is the rotation axis of joint `j` and `p_j` is its position.

**Update step (gradient descent / pseudoinverse):**

```
Δθ = Jᵀ · (J · Jᵀ)⁻¹ · Δp        (Moore-Penrose pseudoinverse)
```

or the simpler **transpose method**:

```
Δθ = α · Jᵀ · Δp
```

where `α` is a step size. This is repeated each frame until `‖p_end − p_target‖ < ε`.

The **pseudoinverse** `J⁺ = Jᵀ(JJᵀ)⁻¹` directly uses concepts from:
- Matrix transpose
- Matrix multiplication
- Matrix inversion (and handling of singular/near-singular matrices via SVD)

---

### 5. Singular Value Decomposition (SVD) for Stability

When the Jacobian is near-singular (arm in a degenerate configuration), we use **SVD** to compute a stable pseudoinverse:

```
J = U · Σ · Vᵀ
J⁺ = V · Σ⁺ · Uᵀ
```

where `Σ⁺` replaces each non-zero singular value `σᵢ` with `1/σᵢ` (and zeros out tiny singular values below a threshold). This prevents numerical blow-up at kinematic singularities.

---

### 6. 3D Perspective Projection

To render the 3D arm on a 2D screen, we apply a **projection matrix**. For perspective projection:

```
P = | f/aspect   0      0              0        |
    |    0        f      0              0        |
    |    0        0   (far+near)/(near-far)   2·far·near/(near-far) |
    |    0        0     -1              0        |
```

where `f = 1/tan(fov/2)`.

The full rendering pipeline per point:

```
p_screen = P · View · p_world
```

After multiplication, we perform **perspective divide**:

```
x_ndc = p_screen.x / p_screen.w
y_ndc = p_screen.y / p_screen.w
```

This maps world coordinates to Normalized Device Coordinates (NDC) in `[-1, 1]`.

**View matrix** (camera transform) is constructed from:
- Camera position `e`
- Look-at target `c`
- Up vector `u`

```
View = | right.x   right.y   right.z   -dot(right, e) |
       |  up.x      up.y      up.z     -dot(up, e)    |
       | -fwd.x   -fwd.y    -fwd.z     dot(fwd, e)    |
       |    0        0         0             1         |
```

This is an orthonormal basis change — a core topic in linear algebra.

---

### 7. Summary of Linear Algebra Concepts Used

| Concept | Where Applied |
|---|---|
| Matrix multiplication | FK chain, projection pipeline, Jacobian update |
| Rotation matrices (SO(3)) | Each joint transformation |
| Homogeneous coordinates | Unified rotation + translation in 4×4 matrices |
| Jacobian matrix | IK — mapping joint velocities to Cartesian velocities |
| Matrix transpose | Jacobian transpose IK method |
| Matrix inversion | Pseudoinverse for IK |
| SVD | Stable pseudoinverse near singularities |
| Orthonormal bases | Camera view matrix construction |
| Dot and cross products | Jacobian column computation |
| Eigenvalues | Analysis of singularities in the Jacobian |

---

## References

- Strang, G. — *Introduction to Linear Algebra*
- Murray, Li, Sastry — *A Mathematical Introduction to Robotic Manipulation*
- Scratchapixel — Perspective Projection Matrix derivation
- Buss, S. — *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods*

---

*Course: Linear Algebra | Project Type: Simulation & Visualization*
