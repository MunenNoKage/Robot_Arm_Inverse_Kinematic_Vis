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

The topic of this project is a 3D robot arm simulator. Every computation in it relies on linear algebra.

The arm consists of segments connected at rotating joints. Each joint has an angle — theta_1, theta_2, theta_3. The tip of the arm is the end-effector; the desired position it must reach is the target.

The fundamental problem: given the target coordinates, determine the joint angles that position the end-effector there. This is inverse kinematics — we know the output and must recover the input.

A second problem is visualization: the arm exists in 3D, but the screen is 2D. Converting between them is also a matrix operation. I will now walk through the mathematics of both.

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

All positions — joints, end-effector, target — are represented as vectors. A 3D point has coordinates x, y, z. We append a fourth component equal to 1, obtaining a 4×1 vector in homogeneous coordinates.

The motivation: in standard coordinates, rotation is a matrix product but translation is an addition. In homogeneous coordinates, both become a single 4×4 matrix multiplication. This unification simplifies the entire pipeline.

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

A rotation by angle θ around the Z-axis is represented by this 3×3 matrix. Analogous matrices handle X and Y rotations. Two structural properties matter:

**Write under the matrix:**
```
Properties:
  R⁻¹ = Rᵀ     (inverse = transpose, very cheap!)
  det(R) = 1    (no stretching, pure rotation)
```

The inverse equals the transpose — computing it requires no arithmetic, just reindexing. The determinant is 1, confirming the transformation is purely rotational with no scaling.

Each joint performs a rotation followed by a translation along the bone. Both are merged into a single 4×4 matrix:

**Write this:**
```
Joint transform (4×4):

Tᵢ = | Rᵢ(θᵢ)  tᵢ |    Rᵢ = 3×3 rotation
     |  0  0  0  1 |    tᵢ = 3×1 translation (bone length)
```

This matrix fully encodes one joint's geometry.

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

Forward kinematics maps joint angles to end-effector position. The per-joint matrices are multiplied left to right, yielding a single accumulated transform T_total. Applying it to the origin gives p_end — the current tip position. The operation is entirely matrix multiplication.

The inverse problem — recovering angles from a desired position — has no general closed-form solution, so we use an iterative method based on the Jacobian matrix.

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

Each iteration computes Δp — the vector from the current tip to the target. The goal is to find a joint angle update Δθ that moves the tip by Δp. This mapping is captured by the Jacobian.

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

J is a 3×n matrix; entry (i, j) is the partial derivative of end-effector coordinate i with respect to joint angle j. The relation Δp ≈ J·Δθ is a first-order linear approximation, valid for small steps. We know Δp; we need Δθ. Since J is rectangular, we solve via the pseudoinverse.

**Write this:**
```
Solving:    Δθ = J⁺ · Δp

Pseudoinverse:   J⁺ = Jᵀ · (J · Jᵀ)⁻¹

This is like solving   Ax = b   when A is not square
→ gives the least-squares best solution
```

**Say:**

J⁺ is the Moore-Penrose pseudoinverse. It yields the minimum-norm least-squares solution to the system J·Δθ = Δp — exactly the generalization of Ax = b to non-square matrices.

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

Per frame: evaluate FK to get p_end, compute Δp, assemble J, apply J⁺ to get Δθ, increment θ, check convergence. The arm converges smoothly to the target. One issue must still be addressed.

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

At certain configurations — such as full extension — J loses rank. Its column space does not span all directions, and the pseudoinverse becomes numerically unstable. We stabilize it using SVD.

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

SVD factorizes J as U·Σ·Vᵀ. Σ is diagonal with singular values σᵢ measuring how well J covers each direction. To form J⁺, we invert each σᵢ, but clamp any near-zero value to zero rather than dividing by it. This produces a stable pseudoinverse regardless of configuration. With kinematics solved, the last step is rendering.

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

Rendering requires two matrix transforms. V is a change of basis from world coordinates to camera coordinates — it is an orthonormal transform. P is the projection matrix that maps 3D camera-space points to homogeneous 2D. Dividing screen coordinates by w implements perspective: distant points map closer together. The entire rendering step is two matrix multiplications followed by a division.

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

Summary of the pipeline. FK evaluates the matrix chain T₁·T₂·T₃ to locate the end-effector. IK drives the arm toward the target by iteratively solving J·Δθ = Δp using the pseudoinverse. SVD stabilizes that solve. The result is projected onto the screen by applying V and P.

All steps reduce to standard linear algebra: matrix products, a rectangular linear system, SVD, and a basis change. The project will deliver a real-time interactive visualization of this pipeline.

Thank you.
