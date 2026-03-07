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

**Say:**

I am presenting a 3D robot arm simulator built on linear algebra.

The arm has a fixed base. Rigid segments extend from it, connected at joints — each joint rotates by an angle. The last segment ends at the end-effector, the operational tip of the arm. A target point is given in 3D space.

The project solves two problems. First, inverse kinematics: compute the joint angles such that the end-effector reaches the target. Second, rendering: project the 3D arm onto a 2D screen. Both reduce to sequences of matrix operations.

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

Each position in the scene is a vector. For a 3D point we have x, y, z. We lift this to 4D by appending 1 — so-called homogeneous coordinates. The reason: a rigid-body transform consists of a rotation and a translation. In ordinary coordinates these require separate operations. In homogeneous coordinates, both are encoded in one 4×4 matrix, and the entire transform is a single multiplication.

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

Rotation around the Z-axis by θ is this 3×3 matrix. Matching matrices exist for X and Y. Two properties are central:

**Write under the matrix:**
```
Properties:
  R⁻¹ = Rᵀ     (inverse = transpose, very cheap!)
  det(R) = 1    (no stretching, pure rotation)
```

Inversion costs nothing — R⁻¹ equals Rᵀ. The determinant equals 1, so lengths and angles are preserved.

At each joint the arm both rotates and advances by the bone length. We package both into one 4×4 homogeneous matrix:

**Write this:**
```
Joint transform (4×4):

Tᵢ = | Rᵢ(θᵢ)  tᵢ |    Rᵢ = 3×3 rotation
     |  0  0  0  1 |    tᵢ = 3×1 translation (bone length)
```

A single matrix captures the complete geometry of one joint.

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

Forward kinematics answers: given angles, where is the tip? We multiply the per-joint transforms in order from base to tip. The product T_total represents the cumulative transform of the whole arm. Multiplying it by the zero vector gives p_end — the current end-effector position in world coordinates. This is a straightforward chain of 4×4 matrix products.

The inverse problem — find the angles given p_end = p_target — cannot be inverted analytically in general, so we linearize and iterate using the Jacobian.

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

We define the residual Δp as the vector from the current end-effector to the target. Each step we want to find a joint angle increment Δθ that moves the tip by Δp. For small increments this relationship is linear, and the coefficient matrix is the Jacobian J.

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

J has 3 rows (one per spatial dimension) and n columns (one per joint). The (i, j) entry is how much end-effector coordinate i changes when angle j increases slightly. The linearized system is Δp = J·Δθ. With Δp known and Δθ unknown, and J non-square, we solve using the pseudoinverse.

**Write this:**
```
Solving:    Δθ = J⁺ · Δp

Pseudoinverse:   J⁺ = Jᵀ · (J · Jᵀ)⁻¹

This is like solving   Ax = b   when A is not square
→ gives the least-squares best solution
```

**Say:**

J⁺ = Jᵀ(JJᵀ)⁻¹ is the right pseudoinverse. It solves Ax = b in the least-squares sense when A is 3×n with n > 3, giving the smallest-norm Δθ consistent with the desired displacement.

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

The loop per frame: run FK to get p_end, subtract from p_target to get Δp, construct J, multiply J⁺·Δp to get Δθ, add Δθ to the current angles, check convergence. The arm advances toward the target each iteration. One numerical difficulty remains.

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

In degenerate configurations — for example the arm fully straightened — J loses rank. Some singular values approach zero, and J⁺ computed naively would amplify noise without bound. SVD provides a numerically stable factorization.

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

SVD writes J = U·Σ·Vᵀ where Σ is diagonal and the σᵢ are non-negative singular values. J⁺ = V·Σ⁺·Uᵀ where Σ⁺ inverts only the singular values above a chosen threshold; the rest map to zero. The result is a bounded, well-conditioned update at every configuration.

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

Display requires mapping 3D world coordinates to 2D pixel coordinates. First, V — the view matrix — performs an orthonormal basis change to express points in camera space. Then P — the projection matrix — encodes the perspective transform. Dividing by the homogeneous w-component applies the perspective divide: points further from the camera produce a larger w and thus appear smaller on screen. The full operation is two 4×4 multiplications and one division per point.

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

To close: the FK chain T₁·T₂·T₃ locates the end-effector. The IK loop solves Δp = J·Δθ at each frame using the SVD-stabilized pseudoinverse and increments θ until convergence. V and P then project the 3D result to the 2D screen.

The complete system is built from vectors, matrix multiplication, a rectangular linear system, SVD, and a change of basis — foundational topics of linear algebra. The goal is a real-time interactive demo where the arm tracks a moving target.

Thank you.
