# Video Script - Robot Arm Inverse Kinematics (≈8 min)

> **How to use:** Read the **speech text** out loud. Everything inside a box (``` ```) is what you **write on paper** - copy it exactly. Draw diagrams as shown.

---

## PAGE 1 - What is this project? (≈1.5 min)

**Write this title:**
```
Robot Arm - Inverse Kinematics
```

**Draw this diagram:**
```
           * target

      O  <- end-effector  (theta_3)
     /
    / L2
   /
  O  (theta_2)
   \
    \ L1
     \
      O  (theta_1)
      |
     [=]  base
```
*(3 circles O = 3 joints. Bottom O = joint 1, middle = joint 2, top = joint 3 / end-effector. Connect with lines and label them L1, L2. Draw small arcs at each joint for theta_1/2/3. Star is the target.)*

Hello. My project is a 3D robot arm simulator, and the whole thing is built on linear algebra.

Let me explain what it does. Here we have a robot arm. It has a fixed base, and connected to it are rigid segments - like bones - joined together at these points, which we call joints. Each joint can rotate by some angle - θ₁, θ₂, θ₃.

The very tip of the arm is called the end-effector. And over here we have a target - some point in space that we want the arm to reach.

So the central question of my project is: given a target position, what should every joint angle be so that the tip of the arm lands exactly on the target? This problem is called **inverse kinematics**. The word "inverse" means we are going backwards - we know the result we want, but we need to find the inputs.

And there is one more challenge: the arm lives in 3D space, but we see it on a flat 2D screen, so we also need projection - which is another matrix operation. Let me now go through the math, piece by piece.

---

## PAGE 2 - How we represent points (≈1 min)

**Write this:**
```
Step 1: Representing Data

A 3D point:     p = [x, y, z]

Homogeneous:    p = | x |    (4×1 vector)
                    | y |
                    | z |
                    | 1 |

Why? → rotation + translation = ONE matrix multiply:

p' = M · p      (4×4) · (4×1) = (4×1)
```

**Say:**

Every position is a 3D vector. We extend it to a 4×1 vector by appending 1 — this is called homogeneous coordinates. The benefit: rotation and translation can now be combined into a single 4×4 matrix multiplication instead of two separate operations. All points and all transforms in this project follow this convention.

---

## PAGE 3 - Rotation matrices and joint transforms (≈1.5 min)

**Write this:**
```
Step 2: Rotation Matrices

Rotation around Z-axis by angle θ:

R_z(θ) = | cosθ  -sinθ   0 |
         | sinθ   cosθ   0 |
         |  0      0     1 |
```

**Draw this next to the matrix:**
```
  y
  |  .p'
  | /
  |/ arc=theta
  +-------> x
          p
```

**Say:**

Each joint rotation is represented by a rotation matrix — here, rotation around Z by angle θ. Equivalent matrices exist for X and Y axes. Two key properties:

**Write under the matrix:**
```
Properties:
  R⁻¹ = Rᵀ     (inverse = transpose, very cheap!)
  det(R) = 1    (no stretching, pure rotation)
```

R inverse equals R transpose — inversion is free. Determinant 1 means no scaling, pure rotation.

Each joint both rotates and translates by the bone length. These are combined into one 4×4 transform matrix:

**Write this:**
```
Joint transform (4×4):

Tᵢ = | Rᵢ(θᵢ)  tᵢ |    Rᵢ = 3×3 rotation
     |  0  0  0  1 |    tᵢ = 3×1 translation (bone length)
```

One 4×4 matrix fully describes one joint: its rotation and translation.

---

## PAGE 4 - Forward Kinematics (≈1 min)

**Draw this chain:**
```
  O --T1(t1)--> O --T2(t2)--> O --T3(t3)--> O
base          j1             j2           tip
```

**Write below it:**
```
Step 3: Forward Kinematics

T_total = T₁(θ₁) · T₂(θ₂) · T₃(θ₃)

                                | 0 |
p_end = T_total ·              | 0 |
                                | 0 |
                                | 1 |
```

**Say:**

Forward kinematics computes the end-effector position from known joint angles. The joint transforms are multiplied in sequence to give one combined matrix T_total. Multiplying by the origin yields the end-effector position — a pure chain of matrix multiplications.

Inverse kinematics is the reverse problem: given the target position, find the angles. This cannot be solved in closed form for general configurations, so we solve it iteratively using the Jacobian.

---

## PAGE 5 - Inverse Kinematics and the Jacobian (≈2 min)

**Write this:**
```
Step 4: Inverse Kinematics

Problem:  find angles θ₁, θ₂, θ₃
          such that  FK(θ) = p_target

Idea: solve ITERATIVELY - move a little closer each step
```

**Draw this:**
```
  * target
  ^
  | Δp  (gap to close)
  |
  O  <- tip
  |\  L2
  O  \  L1
  |   O
 [=] (base)
```

**Say:**

At each iteration, we compute the error vector Δp — the displacement from the current end-effector position to the target. We then ask: what change in joint angles Δθ will produce this displacement? The Jacobian answers this.

**Write this:**
```
The Jacobian (3×n matrix):

J = | ∂x/∂θ₁   ∂x/∂θ₂   ∂x/∂θ₃ |
    | ∂y/∂θ₁   ∂y/∂θ₂   ∂y/∂θ₃ |
    | ∂z/∂θ₁   ∂z/∂θ₂   ∂z/∂θ₃ |

Meaning: "if I nudge angle θⱼ, how does the tip move?"

Key equation:    Δp  ≈  J · Δθ
                (3×1)  (3×n)(n×1)
```

**Say:**

The Jacobian J is a 3×n matrix of partial derivatives. Column j gives the end-effector velocity when joint j rotates. The linear relation Δp ≈ J·Δθ holds for small steps. We know Δp and need Δθ, so we must invert J. Since J is not square, no regular inverse exists — we use the Moore-Penrose pseudoinverse.

**Write this:**
```
Solving:    Δθ = J⁺ · Δp

Pseudoinverse:   J⁺ = Jᵀ · (J · Jᵀ)⁻¹

This is like solving   Ax = b   when A is not square
→ gives the least-squares best solution
```

**Say:**

The pseudoinverse J⁺ solves the least-squares system Ax = b for non-square A. It gives the minimum-norm angle update that best achieves the desired end-effector displacement.

**Write this:**
```
Algorithm (runs every frame):

  1. p_end  = FK(θ)              ← matrix chain
  2. Δp     = p_target − p_end   ← gap vector
  3. J      = build Jacobian     ← partial derivatives
  4. Δθ     = J⁺ · Δp            ← pseudoinverse
  5. θ      = θ + Δθ             ← update angles
  6. if ‖Δp‖ < ε → done         ← close enough?
     else → go to step 1
```

**Say:**

Each frame: compute the current end-effector position via FK, form the error Δp, build J, solve for Δθ via pseudoinverse, update the angles, and repeat until convergence. This produces smooth motion toward the target. However, one numerical problem can arise.

---

## PAGE 6 - Singularities and SVD (≈1 min)

**Draw these two side by side:**
```
  NORMAL:        SINGULAR:

    O               O-O-O-O  <- all in a line!
   / \              |
  O   O            [=]  -> J nearly singular!
  |                     Pseudoinverse blows up!
 [=]
```

**Say:**

When the arm is fully extended, J becomes nearly singular — its rank drops and the pseudoinverse diverges. To handle this, we compute the pseudoinverse via SVD.

**Write this:**
```
SVD:    J = U · Σ · Vᵀ

        Σ = | σ₁  0   0 |    ← singular values
            | 0   σ₂  0 |
            | 0   0   σ₃ |

Stable pseudoinverse:

        J⁺ = V · Σ⁺ · Uᵀ

        Σ⁺: each σᵢ → 1/σᵢ
             BUT if σᵢ ≈ 0 → set to 0 (skip it!)
```

**Say:**

SVD decomposes J into U·Σ·Vᵀ. The diagonal entries of Σ are singular values. In Σ⁺, each 1/σᵢ is kept only if σᵢ is above a threshold; near-zero values are replaced with zero. This damps instability at singular configurations without changing the solution elsewhere. The remaining step is rendering the 3D arm onto the screen.

---

## PAGE 7 - 3D to 2D Projection (≈1 min)

**Draw this:**
```
  ARM (3D)        SCREEN    CAMERA

  O ............. a
   \              |       \
    O ........... b ....... E  (eye)
   /              |       /
  O ............. c
  |               |
 [=]           (2D points)
```
*(3 arm joints on the left. Dotted lines go through the screen to the eye E. Where they cross the screen = projected 2D points a, b, c.)*

**Write below:**
```
Step 5: Projection (3D → 2D)

p_screen = P · V · p_world
           ↑   ↑
           |   └── View matrix: world → camera coords
           |       (orthonormal basis change)
           |
           └────── Projection matrix: 3D → 2D
                   (4×4 matrix)

Then:  x_2D = p_screen.x / p_screen.w
       y_2D = p_screen.y / p_screen.w
              ↑
              perspective divide:
              farther objects → smaller on screen
```

**Say:**

Rendering applies two transforms. The View matrix V is an orthonormal basis change from world space to camera space. The Projection matrix P maps 3D camera coordinates to homogeneous screen coordinates. The final perspective divide by w gives 2D normalized coordinates, where depth encodes apparent size. Two matrix multiplications and one component-wise division.

---

## PAGE 8 - Full Picture and Summary (≈30 sec)

**Draw this flowchart:**
```
                FULL PIPELINE:

  Target (*)                  Angles θ
     │                           │
     ▼                           ▼
  ┌──────────┐  Jacobian  ┌──────────────┐
  │ Inverse  │◄──────────│   Forward     │
  │Kinematics│  (J⁺·Δp)  │  Kinematics  │
  │ find Δθ  │───────────►│  (T₁·T₂·T₃) │
  └──────────┘  update θ  └──────┬───────┘
                                 │
                                 ▼
                          ┌──────────────┐
                          │  Projection  │
                          │  (P · V · p) │
                          └──────┬───────┘
                                 │
                                 ▼
                            2D Screen
```

**Write this summary:**
```
Linear Algebra used:
  ✓ Vectors          - all positions
  ✓ Matrix multiply  - FK chain, projection
  ✓ Rotation matrices - joint transforms
  ✓ Jacobian         - angle → position mapping
  ✓ Pseudoinverse    - solving Ax=b (non-square)
  ✓ SVD              - stability at singularities
  ✓ Basis change     - camera view matrix
```

**Say:**

The full pipeline: Forward kinematics computes end-effector position via a matrix chain. Inverse kinematics iteratively solves for joint angles using the Jacobian pseudoinverse. SVD ensures numerical stability at singular configurations. Projection matrices render the result to screen.

Every operation in this project is a direct application of linear algebra: matrix multiplication, linear systems, SVD, and basis change. The implementation will be a real-time interactive simulation.

Thank you.
