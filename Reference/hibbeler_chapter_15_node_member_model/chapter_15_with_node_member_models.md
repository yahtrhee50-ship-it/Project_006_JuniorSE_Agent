# Chapter 15 - Coordinate-Based Node and Member Models

> Reconstructed from the primary structural diagrams in Chapter 15. The coordinate origin is placed at the leftmost physical node; global +x is to the right and +y is upward. `EI0` denotes the constant flexural rigidity stated in the problem.

## Modeling convention

- Beam element local axis runs from the listed i-node to j-node.
- Standard bending degrees of freedom are transverse displacement `v` and rotation `theta` at each element end.
- Coordinates reproduce geometry and connectivity; support conditions and releases are stated separately.
- For Problem 15-8, coincident nodes are used at the internal hinge so the two end rotations remain independent.
- For Problem 15-10, the book assembles only the two 8-ft interior stiffness members; the overhangs transfer their loads as equivalent nodal actions.

## Problem 15-1 / 15-2

![Coordinate model for Problem 15-1 / 15-2](chapter_15_model_assets/problem_15_1_15_2.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | fixed |
| N2 | 6 | 0 | roller (v=0) |
| N3 | 10 | 0 | fixed |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 6 m | EI0 |
| M2 | N2 | N3 | 4 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-1 / 15-2",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "fixed"
    },
    {
      "id": "N2",
      "x": 6,
      "y": 0,
      "condition": "roller (v=0)"
    },
    {
      "id": "N3",
      "x": 10,
      "y": 0,
      "condition": "fixed"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 6,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 4,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Same geometry for Problems 15-1 and 15-2. Problem 15-2 additionally prescribes +0.005 m vertical support movement at N2.

### Textbook Answers

**Problem 15-1**
- M₁ = 90 kN·m (clockwise)
- M₃ = 22.5 kN·m (clockwise)

**Problem 15-2**
- M₁ = 27.5 kN·m (clockwise)
- M₃ = 116 kN·m (clockwise)

## Problem 15-3

![Coordinate model for Problem 15-3](chapter_15_model_assets/problem_15_3.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | roller, push/pull |
| N2 | 12 | 0 | roller, push/pull |
| N3 | 20 | 0 | roller, push/pull |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 12 m | EI0 |
| M2 | N2 | N3 | 8 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-3",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "roller, push/pull"
    },
    {
      "id": "N2",
      "x": 12,
      "y": 0,
      "condition": "roller, push/pull"
    },
    {
      "id": "N3",
      "x": 20,
      "y": 0,
      "condition": "roller, push/pull"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 12,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 8,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** All members have constant EI. Applied loading includes 6 kN/m on M1 and a 20 kN·m nodal moment at N3 as shown.

### Textbook Answers

- R₃ = 7.85 kN ↑ (roller tension)
- R₄ = 40.2 kN ↑
- M₅ = 86.6 kN·m (clockwise)
- R₆ = 39.6 kN ↑

## Problem 15-4

![Coordinate model for Problem 15-4](chapter_15_model_assets/problem_15_4.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | pin |
| N2 | 10 | 0 | roller, push/pull |
| N3 | 20 | 0 | roller, push/pull |
| N4 | 30 | 0 | free end |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 10 ft | EI0 |
| M2 | N2 | N3 | 10 ft | EI0 |
| M3 | N3 | N4 | 10 ft | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-4",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "ft"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "pin"
    },
    {
      "id": "N2",
      "x": 10,
      "y": 0,
      "condition": "roller, push/pull"
    },
    {
      "id": "N3",
      "x": 20,
      "y": 0,
      "condition": "roller, push/pull"
    },
    {
      "id": "N4",
      "x": 30,
      "y": 0,
      "condition": "free end"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 10,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 10,
      "EI": "EI0"
    },
    {
      "id": "M3",
      "i": "N3",
      "j": "N4",
      "L": 10,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Constant EI throughout; 3-k downward point load at N4.

### Textbook Answers

- R₆ = 6.75 k ↑
- R₇ = 4.5 k ↓ (roller tension)
- R₈ = 0.75 k ↓ (roller tension)

## Problem 15-5

![Coordinate model for Problem 15-5](chapter_15_model_assets/problem_15_5.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | pin |
| N2 | 6 | 0 | roller |
| N3 | 14 | 0 | roller |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 6 m | EI0 |
| M2 | N2 | N3 | 8 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-5",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "pin"
    },
    {
      "id": "N2",
      "x": 6,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N3",
      "x": 14,
      "y": 0,
      "condition": "roller"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 6,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 8,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Constant EI. Triangular distributed load on M1 varies from 0 at N1 to 15 kN/m at N2.

### Textbook Answers

- R₄ = 1.93 kN ↓ (roller tension)
- R₅ = 34.5 kN ↑
- R₆ = 12.4 kN ↑

## Problem 15-6

![Coordinate model for Problem 15-6](chapter_15_model_assets/problem_15_6.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | fixed |
| N2 | 6 | 0 | roller |
| N3 | 14 | 0 | roller |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 6 m | EI0 |
| M2 | N2 | N3 | 8 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-6",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "fixed"
    },
    {
      "id": "N2",
      "x": 6,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N3",
      "x": 14,
      "y": 0,
      "condition": "roller"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 6,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 8,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Constant EI. Uniform 10 kN/m load acts on M2.

### Textbook Answers

- R₃ = 32.25 kN ↑
- R₄ = 85.75 kN ↑
- R₅ = 22.0 kN ↑
- M₆ = 14.0 kN·m (counterclockwise)

## Problem 15-7

![Coordinate model for Problem 15-7](chapter_15_model_assets/problem_15_7.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | fixed |
| N2 | 6 | 0 | roller |
| N3 | 10 | 0 | fixed |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 6 m | EI0 |
| M2 | N2 | N3 | 4 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-7",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "fixed"
    },
    {
      "id": "N2",
      "x": 6,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N3",
      "x": 10,
      "y": 0,
      "condition": "fixed"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 6,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 4,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Constant EI. Uniform loads: 9 kN/m on M1 and 6 kN/m on M2.

### Textbook Answers

- R₂ = 41.4 kN ↑
- R₃ = 7.73 kN ↑
- M₄ = 2.30 kN·m (clockwise)
- R₅ = 28.9 kN ↑
- M₆ = 30.8 kN·m (clockwise)

## Problem 15-8

![Coordinate model for Problem 15-8](chapter_15_model_assets/problem_15_8.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | fixed |
| N2L | 4 | 0 | internal hinge, left face |
| N2R | 4 | 0 | internal hinge, right face |
| N3 | 7 | 0 | roller |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2L | 4 m | EI0 |
| M2 | N2R | N3 | 3 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-8",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "fixed"
    },
    {
      "id": "N2L",
      "x": 4,
      "y": 0,
      "condition": "internal hinge, left face"
    },
    {
      "id": "N2R",
      "x": 4,
      "y": 0,
      "condition": "internal hinge, right face"
    },
    {
      "id": "N3",
      "x": 7,
      "y": 0,
      "condition": "roller"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2L",
      "L": 4,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2R",
      "j": "N3",
      "L": 3,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** N2L and N2R are coincident. Tie their vertical translations; keep their rotations independent to model the internal hinge. Triangular load on M2 varies from 15 kN/m at N2 to 0 at N3.

### Textbook Answers

- R₅ = 7.50 kN ↑
- R₆ = 15.0 kN ↑
- M₇ = 60.0 kN·m (counterclockwise)

## Problem 15-9

![Coordinate model for Problem 15-9](chapter_15_model_assets/problem_15_9.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | roller |
| N2 | 12 | 0 | roller |
| N3 | 24 | 0 | roller |
| N4 | 36 | 0 | pin |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 12 m | EI0 |
| M2 | N2 | N3 | 12 m | EI0 |
| M3 | N3 | N4 | 12 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-9",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N2",
      "x": 12,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N3",
      "x": 24,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N4",
      "x": 36,
      "y": 0,
      "condition": "pin"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 12,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 12,
      "EI": "EI0"
    },
    {
      "id": "M3",
      "i": "N3",
      "j": "N4",
      "L": 12,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Constant EI. Uniform 4 kN/m load over all three spans.

### Textbook Answers

- M₁ = M₄ = 0
- M₂ = M₃ = 44.2 kN·m

## Problem 15-10

![Coordinate model for Problem 15-10](chapter_15_model_assets/problem_15_10.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N0 | 0 | 0 | free overhang end |
| N1 | 4 | 0 | roller |
| N2 | 12 | 0 | pin |
| N3 | 20 | 0 | roller |
| N4 | 24 | 0 | free overhang end |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| OH-L | N0 | N1 | 4 ft | EI0 |
| M1 | N1 | N2 | 8 ft | EI0 |
| M2 | N2 | N3 | 8 ft | EI0 |
| OH-R | N3 | N4 | 4 ft | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-10",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "ft"
  },
  "nodes": [
    {
      "id": "N0",
      "x": 0,
      "y": 0,
      "condition": "free overhang end"
    },
    {
      "id": "N1",
      "x": 4,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N2",
      "x": 12,
      "y": 0,
      "condition": "pin"
    },
    {
      "id": "N3",
      "x": 20,
      "y": 0,
      "condition": "roller"
    },
    {
      "id": "N4",
      "x": 24,
      "y": 0,
      "condition": "free overhang end"
    }
  ],
  "members": [
    {
      "id": "OH-L",
      "i": "N0",
      "j": "N1",
      "L": 4,
      "EI": "EI0"
    },
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 8,
      "EI": "EI0"
    },
    {
      "id": "M2",
      "i": "N2",
      "j": "N3",
      "L": 8,
      "EI": "EI0"
    },
    {
      "id": "OH-R",
      "i": "N3",
      "j": "N4",
      "L": 4,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** The chapter assembles stiffness only for M1 and M2. The 4-ft overhangs are represented through equivalent nodal loads at N1 and N3. EI is constant over the physical beam.

### Textbook Answers

- R₁ = 25.5 k ↑
- R₂ = 21.0 k ↑
- R₃ = 25.5 k ↑

## Problem 15-11

![Coordinate model for Problem 15-11](chapter_15_model_assets/problem_15_11.svg)

### Nodes

| Node | x | y | Support / release condition |
|---|---:|---:|---|
| N1 | 0 | 0 | fixed |
| N2 | 4 | 0 | smooth vertical slider: rotation restrained, vertical translation free |

### Members

| Member | i-node | j-node | Length | Flexural rigidity |
|---|---|---|---:|---|
| M1 | N1 | N2 | 4 m | EI0 |

### Machine-readable topology

```json
{
  "problem": "15-11",
  "coordinate_system": {
    "x": "right",
    "y": "up",
    "units": "m"
  },
  "nodes": [
    {
      "id": "N1",
      "x": 0,
      "y": 0,
      "condition": "fixed"
    },
    {
      "id": "N2",
      "x": 4,
      "y": 0,
      "condition": "smooth vertical slider: rotation restrained, vertical translation free"
    }
  ],
  "members": [
    {
      "id": "M1",
      "i": "N1",
      "j": "N2",
      "L": 4,
      "EI": "EI0"
    }
  ]
}
```

**Modeling note:** Constant EI. Uniform 30 kN/m load on M1. In the bending model, v at N2 is free and θ at N2 is restrained.

### Textbook Answers

- M₂ = 80 kN·m (counterclockwise)
- R₃ = 120 kN ↑
- M₄ = 160 kN·m (counterclockwise)


---

# Original Chapter 15 Markdown Content

# Chapter 15

> Source: *Hibbeler Structural Analysis, 8th Edition - Solutions*. PDF pages 518-535.

> Each section includes a rendered page for faithful equations/figures, followed by searchable text extracted from the PDF.


## PDF Page 518

![PDF page 518](chapter_15_assets/page_518.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–1.  Determine the moments at  1 and  3  . Assume  2               5                             4                   6
   is a roller and  1 and  3  are fixed. EI is constant.
                                                                                    25 kN/m

                                                                           2                              1               3

                                                                           1       1                       2      2       3

                                                                                       6 m                       4 m


  Member Stiffness Matrices. For member         ƒ 1          ƒ ,
    12EI   12EI               6EI   6EI
         =      = 0.05556EI        =     = 0.16667EI      L3       63                  L2      62

    4EI   4EI                2EI   2EI
        =     = 0.66667EI         =     = 0.33333EI    L      6               L      6

                  5         2         4         1
                0.05556    0.16667   -0.05556   0.16667   5
     k1 = EI   0.16667    0.66667   -0.16667   0.33333   2             D                                        T
               -0.05556   -0.16667   0.05556   -0.16667  4
                0.16667    0.33333   -0.16667   0.66667   1
  For member         ƒ 2          ƒ ,
    12EI   12EI               6EI   6EI
         =      = 0.1875EI        =     = 0.375EI      L3       43                  L2      42

    4EI   4EI                2EI   2EI
        =     = EI               =     = 0.5EI    L      4               L      4

                4        1        6       3
               0.1875    0.375    -0.1875   0.375   4
     k2 = EI  0.375      1.00     -0.375    0.5     1             D                                  T
               -0.1875   -0.375   0.1875    -0.375  6
                0.375      0.5      -0.375    1.00    3

  Known Nodal Loads and Deflection. The nodal load acting on the unconstrained
  degree of freedom (Code number 1) is shown in Fig. a.Thus;
                               0  2
                               0  3
    Qk = [75] 1   and  Dk = E 0 U 4
                               0  5
                               0  6

  Load-Displacement Relation. The structure stiffness matrix is a 6 * 6 matrix
  since the highest Code number is 6.Applying Q = KD

                 1         2        3        4         5         6
    75          1.6667    0.33333      0.5     0.20833    0.16667    -0.375  1  D1
   Q2         0.33333   0.66667      0     -0.16667   0.16667      0     2   0
   Q3                                                                           0                                                                        3      = EI                   0.5        0        1.00      0.375        0       -0.375  F      V                                                                             V             F                                                                      V                                                                         F
   Q4         0.20833   -0.16667    0.375    0.24306   -0.05556   -0.1875  4   0
   Q5         0.16667   0.16667      0     -0.05556   0.05556      0     5   0
   Q6         -0.375      0      -0.375   -0.1875      0       0.1875   6   0


  From the matrix partition, Qk = K11Du + K12Dk,

                                  45
     75 = 1.66667EID1 + 0   D1 =                           EI

                                                  517

```

</details>


## PDF Page 519

![PDF page 519](chapter_15_assets/page_519.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–1.  Continued



  Also, Qu = K21Du + K22Dk,

                    45    Q2 = 0.33333EIa   b + 0 = 15 kN  # m                EI

                45    Q3 = 0.5EIa   b + 0 = 22.5 kN  # m             EI

  Superposition of these results and the (FEM) in Fig. b,
   M1 = 15 + 75 = 90 kN  # m d                                       Ans.
   M3 = 22.5 + 0 = 22.5 kN  # m d                                     Ans.





  15–2.  Determine the moments at  1 and  3  if the support              5                             4                   6
    2 moves upward 5 mm. Assume  2  is a roller and  1 and
                                                                                    25 kN/m    3  are fixed. EI = 60(106) N  # m2 .
                                                                          2                              1               3

                                                                           1       1                       2      2       3

                                                                                      6 m                       4 m


  Member Stiffness Matrices. For member         ƒ 1          ƒ ,
    12EI   12EI               6EI   6EI
         =      = 0.05556 EI       =     = 0.16667 EI      L3       63                  L2      62

    4EI   4EI                2EI   2EI
        =     = 0.66667EI         =     = 0.33333 EI    L      6               L      6
                 5         2         4         1
     k1 = EI   0.05556    0.16667   -0.05556   0.16667   5
                0.16667    0.66667   -0.16667   0.33333   2             D                                        T
              -0.05556   -0.16667   0.05556   -0.16667  4
                0.16667    0.33333   -0.16667   0.66667   1
  For member         ƒ 2          ƒ ,
    12EI   12EI               6EI   6EI
         =      = 0.1875EI        =     = 0.375EI      L3       43                  L2      42

    4EI   4EI                2EI   2EI
        =     = EI               =     = 0.5EI    L      4               L      4





                                                 518

```

</details>


## PDF Page 520

![PDF page 520](chapter_15_assets/page_520.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–2.  Continued





                 4        1        6        3
                0.1875    0.375   -0.1875    0.375   4
     k2 = EI    0.375      1.00    -0.375      0.5    1             D                                  T
               -0.1875   -0.375   0.1875   -0.375  6
                 0.375       0.5     -0.375     1.00   3

  Known Nodal Loads and Deflection. The nodal load acting on the unconstrained
  degree of freedoom (code number 1) is shown in Fig. a.Thus,

                                   0     2
                                   0     3
    Qk = [75(103)] 1   and  Dk = E 0.005 U 4
                                   0     5
                                   0     6

  Load-Displacement Relation. The structure stiffness matrix is a 6 * 6 matrix since
  the highest code number is 6.Applying Q = kD

                     1         2        3        4         5         6
    75(103)         1.66667   0.33333      0.5     0.20833    0.16667    -0.375  1   D1
     Q2            0.33333   0.66667      0     -0.16667   0.16667      0     2    0
     Q3               0.5        0        1.00      0.375        0       -0.375  3    0  F       V = EI F                                                        V  F     V
     Q4            0.20833   -0.16667    0.375    0.24306   -0.05556   -0.1875  4  0.005
     Q5            0.16667   0.16667      0     -0.05556   0.05556      0     5    0
     Q6           -0.375      0      -0.375   -0.1875      0       0.1875   6    0

  From the matrix partition, Qk = K11Du + K12Dk,
     75(103) = [1.6667D1 + 0.20833(0.005)][60(106)]
    D1 = 0.125(10-3) rad
  Using this result and apply, Qu = K21Du + K22Dk,
    Q2 = {0.33333[0.125(10-3)] + (-0.16667)(0.005)}[60(106)] = -47.5 kN  # m
    Q3 = {0.5[0.125(10-3)] + 0.375(0.005)}[60(106)] = 116.25 kN  # m

  Superposition these results to the (FEM) in Fig. b,
 M1 = -47.5 + 75 = 27.5 kN  # m                                       Ans.
 M3 = 116.25 + 0 = 116.25 kN.m = 116 kN  # m                           Ans.





                                                  519

```

</details>


## PDF Page 521

![PDF page 521](chapter_15_assets/page_521.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–3.  Determine the reactions at the supports. Assume                        6                        4             3
  the rollers can either push or pull on the beam. EI  is                                 6 kN/m
  constant.                                                                           5                         2   20 kN!m   1

                                                                                                1                  2
                                                                                      1                        2              3
                                                                                               12 m                8 m

 Member Stiffness Matrices. For member         ƒ 1          ƒ ,
    12EI   12EI               6EI   6EI
         =      = 0.006944EI       =     = 0.041667EI     L3      123                 L2     122

    4EI   4EI                2EI   2EI
        =     = 0.333333EI        =     = 0.166667EI    L     12              L     12
                  6          5          4          2
                0.006944    0.041667   -0.006944   0.041667   6
                                                           5                0.041667    0.333333   -0.041667   0.166667     k1 = EI D                                                          T
              -0.006944   -0.041667   0.006944   -0.041667  4
                0.041667    0.166667   -0.041667   0.333333   2

  For member         ƒ 2          ƒ ,
    12EI   12EI               6EI   6EI
         =      = 0.0234375EI      =     = 0.09375EI     L3       83                  L2      82

    4EI   4EI                2EI   2EI
        =     = 0.5EI             =     = 0.25EI    L      8               L      8

                  4          2          3          1
               0.0234375    0.09375   -0.0234375   0.09375   4    0
                                                           2                                                                87                 0.09375        0.5      -0.09375       0.25     k2 = EI D                                                         T                                                             D                                                                   T
              -0.0234375   -0.09375   0.0234375   -0.09375  3    0
                 0.09375       0.25      -0.09375       0.5     1  -3.76


 Known Nodal Loads And Deflection. The nodal loads acting on the unconstrained
  degree of freedoom (code number 1 and 2) are shown in Fig. a.Thus,

                                0  3
    Qk = c20  1   and  Dk = D 0 T 4           72d 2                0  5
                                0  6

  Load-Displacement Relation. The structure stiffness matrix is a 6 * 6 matrix since
  the highest code number is 6.Applying Q = KD
                  1         2          3           4           5          6
    20              0.5         0.25      -0.09375      0.09375        0          0      1  D1
    72             0.25     0.833333    -0.09375     0.052083     0.166667    0.041667   2  D2
   Q3         -0.09375   -0.09375   0.0234375   -0.0234375      0          0      3   0  F   V = EI F                                                                   V  F   V
   Q4           0.09375    0.052083   -0.0234375   0.0303815   -0.041667   -0.006944  4   0
   Q5             0       0.166667       0        -0.041667    0.333333    0.041667   5   0
   Q6            0       0.041667       0        -0.006944    0.041667    0.006944   6   0

 From the matrix partition, Qk = K11Du + K12Dk,
    20 = EI[0.5D1 + 0.25D2]                                                    (1)
    72 = EI[0.25D1 + 0.833333D2]                                              (2)




                                                 520

```

</details>


## PDF Page 522

![PDF page 522](chapter_15_assets/page_522.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–3.  Continued




  Solving Eqs. (1) and (2),

             3.7647            87.5294
    D1 = -          D2 =           EI            EI

  Also, Qu = K21Du + K22Dk
      Q3         -0.09375   -0.09375                    0
      Q4          0.09375    0.052083   1   -3.7647      0
     D   T = EI D                   T        c              d + D  T
      Q5         0          0.166667  EI   87.5294       0
      Q6         0          0.041667                     0

    Q3 = -0.09375(-3.7647) + (-0.09375)(87.5294) = -7.853 kN
    Q4 = 0.09375(-3.7647) + 0.052083(87.5294) = 4.206 kN
    Q5 = 0 + 0.166667(87.5294) =  14.59 kN  # m
    Q6 = 0 + 0.041667(87.5294) = 3.647 kN

  Superposition these results with the (FEM) in Fig. b,

    R3 = -7.853 + 0 = -7.853 kN = 7.85 kN T                          Ans.
    R4 = 4.206 + 36 = 40.21 kN = 40.2 kN c                             Ans.
    M5 = 14.59 + 72 = 86.59 kN  # m = 86.6 kN  # m c                      Ans.
    R6 = 3.647 + 36 = 39.64 kN = 39.6 kN c                             Ans.





  *15–4.  Determine the reactions at the supports. Assume                                                              3 k                                                                               8             7               6                5
     1  is a pin and  2 and  3  are rollers that can either push or
                                                                                  1              2              3              4   pull on the beam. EI is constant.

                                                                                   1    1        2    2         3     3       4


                                                                                      10 ft           10 ft          10 ft

  Member Stiffness Matrices. For member         ƒ 1          ƒ ,        ƒ 2          ƒ and         ƒ 3          ƒ ,
    12EI   12EI          6EI   6EI
         =      = 0.012       =     = 0.06      L3      103             L2     102

    4EI   4EI            2EI   2EI
        =     = 0.4           =     = 0.2    L     10           L     10

                  8       1       7       2
                  0.012     0.06   -0.012    0.06  8
     k1 = EI    0.06      0.4     -0.06     0.2   1              D                              T
                -0.012   -0.06    0.012    -0.06 7
                   0.06      0.2     -0.06     0.4   2





                                                  521

```

</details>


## PDF Page 523

![PDF page 523](chapter_15_assets/page_523.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–4.  Continued


                  7       2       6       3
                  0.012    0.06    -0.012    0.06   7
     k2 = EI    0.06      0.4     -0.06     0.2   2              D                              T
                -0.012   -0.06    0.012    -0.06  6
                   0.06      0.2     -0.06     0.4   3
                  6        3         5         4
                   0.012      0.06    - 0.012     0.06    6
     k3 = EI    0.06        0.4     - 0.06      0.2    3              D                                    T
              - 0.012   - 0.06     0.012    - 0.06  5
                    0.06        0.2     - 0.06      0.4    4

 Known Nodal Load and Deflection. The nodal loads acting on the unconstrained
  degree of freedom (code number 1, 2, 3, 4 and 5 ) is

             0   1
             0   2             0  6
    Qk = E 0 U 3 and Dk = C 0 S 7
             0   4             0  8
          -3  5
  Load-Displacement Relation. The structure stiffness matrix is a 8 * 8 matrix since
  the highest code number is 8.Applying Q = KD

                  1      2      3      4       5       6       7       8
     0                     0.4      0.2      0      0       0       0      -0.06     0.06   1  D1
     0                     0.2      0.8      0.2      0       0      -0.06     0       0.06   2  D2
     0                  0       0.2      0.8      0.2    -0.06     0       0.06      0    3  D3
     0
  H    X = EI H  0      0       0.2      0.4    -0.06     0.06      0       0   X 4 H D4 X
   - 3           0      0     -0.06   -0.06    0.012   -0.012     0       0    5  D5
    Q6            0     -0.06    0      0.06   -0.012    0.024   -0.012     0    6   0
    Q7          -0.06    0      0.06     0       0     -0.012    0.024   -0.012  7   0
    Q8            0.06     0.06     0      0       0       0     -0.012    0.012   8   0



 From the matrix partition, Qk =  k11 Du +  k12 Dk ,
    0 = 04D1 + 0.2D2                                                                         (1)
    0 = 0.2D1 + 0.8D2 + 0.2D3                                                              (2)
    0 = 0.2D2 + 0.8D3 + 0.2D4 - 0.06D5                                                   (3)
    0 = 0.2D3 + 0.4D4 – 0.06D5                                                                (4)
    -3 = -0.06D3 - 0.06D4 + 0.012D5                                                      (5)

  Solving Eq. (1) to (5)

    D1 =  -12.5  D2 = 25  D3 =  -87.5  D4 =  -237.5  D5 = -1875
  Using these results, Qu = K21 Du +  k22 Dk
    Q6 =  6.75 kN                                                                 Ans.
    Q7 =  -4.5 kN                                                                Ans.
    Q8 =  -0.75 kN                                                               Ans.




                                                 522

```

</details>


## PDF Page 524

![PDF page 524](chapter_15_assets/page_524.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–5.  Determine  the  support  reactions. Assume 2 and                                6      15 kN/m 5               4
    3  are rollers and  1  is a pin. EI is constant.                                                  1             2                3

  Member Stiffness Matrices. For member         ƒ 1          ƒ                                                1      1        2     2          3
    12EI   12EI               6EI   6EI
         =      = 0.05556EI         =     = 0.16667EI                                   6 m            8 m      L3       63                   L2      62

    4EI   4EI                 2EI   2EI
        =     = 0.066667EI          =     = 0.033333EI    L     6                L     6

                  6         1          5          2
                 0.05556    0.16667   - 0.05556    0.16667   6
     k1 = EI   0.16667    0.66667    -0.16667    0.33333   1              D                                          T
               -0.05556   -0.16667    0.05556    -0.16667  5
                 0.16667    0.33333    -0.16667    0.66667   2
  For Member         ƒ 2          ƒ ,
    12EI   12EI                6EI   6EI
         =      = 0.0234375EI        =     = 0.09375EI      L3       83                   L2      82

    4EI   4EI                  2EI   2EI
        =     = 0.5EI              =     = 0.025EI    L     8                L     8

                   5          2          4          3
                 0.0234375    0.09375   -0.0234375   0.09375   5
     k2 = EI    0.09375        0.5      -0.09375       0.25    2              D                                            T
               -0.0234375   -0.09375   0.0234375   -0.09375  4
                  0.09375       0.25      -0.09375       0.5     3

  Known Nodal Load and Deflection. The nodal loads acting on the unconstrained
  degree of freedom (code number 1, 2, and 3) are shown in Fig. a

             0   1                 0  4
    Qk = C 36 S 2   and  Dk = C 0 S 5
             0   3                 0  6

  Load-Displacement Relation. The structure stiffness matrix is a 6 * 6 matirx
  since the highest code number is 6.Applying Q = KD ,
                   1         2         3          4           5          6
    0            0.66667    0.33333       0          0         0.16667     0.16667   1  D1
    36            0.33333    1.16667      0.25      -0.09375     -0.07292     0.16667   2  D2
    0              0         0.25         0.5      -0.09375      0.09375        0     3  D3  F   V = EI F                                                                 V  F   V
   Q4             0      -0.09375   -0.09375   0.0234375   -0.0234375      0     4   0
   Q5          -0.16667   -0.07292   0.09375   -0.0234375   0.0789931   -0.05556  5   0
   Q6           0.16667    0.16667       0          0        -0.05556     0.05556   6   0

  From the matrix partition, Qk =  k11 Du +  k12 Dk
      0 = 0.66667D1 + 0.33333D2                     (1)
     36 = 0.33333D1 + 1.16667D2 + 0.25D3         (2)
      0 = 0.25D2 + 0.5D3                             (3)





                                                  523

```

</details>


## PDF Page 525

![PDF page 525](chapter_15_assets/page_525.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–5.  Continued



  Solving Eqs. (1) to (3),

           -20.5714
    D1 =           EI

            41.1429
    D2 =          EI

           -20.5714
    D3 =           EI

  Using these results and apply Qu =  k21 Du + +k22 Dk
                            41.1429                     20.5714
  Q4 = 0 + (-0.09375EI) a       b + (-0.09375EI)a -       b =  -1.929 kN                      EI                   EI

                                                   41.1429  Q5 =  -0.16667EI a -20.5714 b + (-0.07292EI) a       b                   EI                  EI

                          20.5714
        + 0.09375EI a -       b                     EI

     =  -1.500 kN

                                              41.1429  Q6 = 0.16667EI a -20.5714 b + 0.16667EI a       b =  3.429 kN                  EI               EI

  Superposition these results with the FEM show in Fig. b

    R4 =  -1.929 + 0 =  -1.929 kN =  1.93 kN T                           Ans.
    R5 =  -1.500 + 36 =  34.5 kN c
    R6 =  3.429 + 9 =  12.43 kN =  12.4 kN c                             Ans.





                                                 524

```

</details>


## PDF Page 526

![PDF page 526](chapter_15_assets/page_526.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–6.  Determine the reactions at the supports.Assume  1                  5                4      10 kN/m          3
   is fixed  2 and  3  are rollers. EI is constant.                                                                               6                 1
                                                                                                                          2

                                                                                    1           2          2             3                                                                               1
                                                                                     6 m                 8 m



  Member Stiffness Matrices. For member  1  ,

    12EI   12EI               6EI   6EI
         =      = 0.05556EI        =     = 0.16667EI      L3       63                  L2      62

    4EI   4EI                2EI   2EI
        =     = 0.066667EI         =     = 0.33333EI    L     6               L     6

                   5         6         4         1
                  0.05556    0.16667   -0.05556   0.16667   5
     k1 = EI
                  0.16667    0.66667   -0.16667   0.33333   6
              D                                        T
                -0.05556   -0.16667   0.05556   -0.16667  4
                  0.16667    0.33333   -0.16667   0.66667   1

  For Member  2  ,

    12EI   12EI               6EI   8EI
         =      = 0.0234375EI      =     = 0.09375EI      L3       83                  L2      82

    4EI   4EI                2EI   2EI
        =     = 0.5EI             =     = 0.025EI    L     8               L     8

                    4          1          3          2
                 0.0234375    0.09375   -0.0234375   0.09375   4
     k2 = EI
                  0.09375        0.5      -0.09375       0.25    1
              D                                            T
                -0.0234375   -0.09375   0.0234375   -0.09375  3
                  0.09375       0.25      -0.09375       0.5     2

  Known Nodal Load and Deflections. The nodal loads acting on the unconstrained
  degree of freedom (code number 1 and 2) are shown in Fig. a

                                  0  3
           -50 1                0  4
    Qk =   c       d    and  Dk = D T              0  2                0  5
                                  0  6

  Load-Displacement Relation. The structure stiffness matrix is a 6 * 6 matirx
   since the highest code number is 6.Applying Q = KD ,

                    1         2          3           4          5         6
   -50          1.16667      0.25      -0.09375     -0.07292     0.16667    0.33333   1  D1
     0              0.25         0.5      -0.09375      0.09375        0         0     2  D2
    Q3          -0.09375   -0.09375   0.0234375   -0.0234375      0         0     3   0  F    V = EI F                                                                 V  F   V
    Q4          -0.07292   0.09375   -0.0234375   0.0789931   -0.05556   -0.16667  4   0
    Q5           0.16667       0          0        -0.05556     0.05556    0.16667   5   0
    Q6           0.33333       0          0        -0.16667     0.16667    0.66667   6   0





                                                  525

```

</details>


## PDF Page 527

![PDF page 527](chapter_15_assets/page_527.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–6.  Continued




  From the matrix partition, Qk = K11 Du + K12 Dk
    -50 = EI(1.16667D1 + 0.25D2)
       0 = EI (0.25D1 + 0.5D2)

  Solving Eqs. (1) and (2),

           48           24
    D1 =       D2 =         EI         EI

  Using these results and apply in Qu = K21 Du + K22 Dk

                        48                   24
    Q3 = -0.09375EI a-   b + (-0.09375EI)a   b + 0 = 2.25 kN                   EI              EI

                        48                24
    Q4 = -0.07292EI a-   b + 0.09375EI a   b + 0 = 5.75 kN                   EI            EI

                      48
    Q5 = 0.16667EIa-   b + 0 + 0 = -8.00 kN                 EI

                       48
    Q6 = (0.33333EI)a-   b + 0 + 0 = -16.0 kN                  EI

  Superposition these results with the FEM show in Fig. b
    R3 = 2.25 + 30 = 32.25 kN c                                       Ans.
    R4 = 5.75 + 30 + 50 = 85.75 kN c                                  Ans.
    R5 =  - 8.00 + 30 = 22.0 kN c                                     Ans.
    R6 = -16.0 + 30 = 14.0 kN  # m d                                    Ans.





                                                 526

```

</details>


## PDF Page 528

![PDF page 528](chapter_15_assets/page_528.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–7.  Determine the reactions at the supports.Assume  1              5                             2                   3
  and  3 are fixed and  2  is a roller. EI is constant.                                  9 kN/m
                                                                                                             6 kN/m
                                                                           6                            1
                                                                                                                               4
                                                                                       1                   2     2       3                                                                           1
                                                                                        6 m                     4 m



  Member Stiffness Matrices. For member  1  ,

    12EI   12EI               6EI   6EI
         =      = 0.05556EI        =     = 0.16667EI      L3       63                  L2      62

    4EI   4EI                2EI   2EI
        =     = 0.66667EI         =     = 0.33333EI    L      6               L      6

                   5         6         2         1
                 0.05556    0.16667   -0.05556   0.16667   5
     k1 = EI
                 0.16667    0.66667   -0.16667   0.33333   6
              D                                        T
                -0.05556   -0.16667   0.05556   -0.16667  2
                 0.16667    0.33333   -0.16667   0.66667   1

  For member  2  ,

    12EI   12EI               6EI   6EI
         =      = 0.1875EI        =     = 0.375EI      L3       43                  L2      42

     4EI   4EI                2EI   2EI
        =     = EI              =     = 0.5EI     L      4               L      4

                  2        1        3        4
                 0.1875    0.375   -0.1875    0.375   2
                  0.375      1.00    -0.375      0.5    1
     k2 = EI D                                  T
                -0.1875   -0.375   0.1875   -0.375  3
                  0.375       0.5     -0.375     1.00   4


  Known Nodal Loads and Deflections. The nodal load acting on the unconstrained
  degree of freedom (code number 1) are shown in Fig. a

                                0  2
                                0  3
    Qk = [19] 1   and  Dk = E 0 U 4
                                0  5
                                0  6





                                                  527

```

</details>


## PDF Page 529

![PDF page 529](chapter_15_assets/page_529.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–7.  Continued




  Load-Displacement Relation. The structure stiffness matrix is a 6 * 6 matirx
  since the highest code number is 6.Applying Q = KD ,

                 1         2         3        4        5         6
    19          1.66667   0.20833    -0.375      0.5     0.16667    0.33333   1  D1
   Q2          0.20833   0.24306   -0.1875    0.375   -0.05556   -0.16667  2   0
   Q3         -0.375   -0.1875    0.1875   -0.375      0         0     3   0  F   V = EI F                                                        V  F   V
   Q4             0.5       0.375     -0.375     1.00       0         0     4   0
   Q5          0.16667   -0.05556     0        0      0.05556    0.16667   5   0
   Q6          0.33333   -0.16667     0        0      0.16667    0.66667   6   0




 From the matrix partition, Qk = K11Du + K12Dk

                                  11.4
    19 = 1.66667EID1   D1 =                        EI

  Using this result and applying Qu = K21Du + K22Dk
    Q2 = 0.20833EIa11.4 b = 2.375 kN                 EI
    Q3 = -0.375EIa11.4 b = -4.275 kN                EI
    Q4 = 0.5EIa11.4 b = 5.70 kN  # m             EI
    Q5 = 0.16667a11.4 b = 1.90 kN               EI
    Q6 = 0.33333a11.4 b = 3.80 kN  # m               EI

  Superposition these results with the FEM shown in Fig. b,
    R2 = 2.375 + 27 + 12 = 41.375 kN = 41.4 kN c                      Ans.
    R3 = -4.275 + 12 = 7.725 kN c                                    Ans.
    R4 = 5.70 - 8 = -2.30 kN  # m = 2.30 kN.m b                        Ans.
    R5 = 1.90 + 27 = 28.9 kN c                                        Ans.
    R6 = 3.80 + 27 = 30.8 kN.m c                                      Ans.





                                                 528

```

</details>


## PDF Page 530

![PDF page 530](chapter_15_assets/page_530.png)

<details>
<summary>Searchable extracted text</summary>

```text

*15–8.  Determine the reactions at the supports. EI  is                                        4                                                                             6                                         5
  constant.                                                                                          15 kN/m
                                                                               7                     2                   1

                                                                                       1                     2
                                                                               1                   3   2                   3
                                                                                       4 m                   3 m

  Member Stiffness Matrices. For member  1

    12EI   12EI               6EI   6EI
         =      = 0.1875EI        =     = 0.375EI      L3       43                  L2      42

    4EI   4EI                2EI   2EI
        =     = EI               =     = 0.5EI    L      4               L      4

                  6        7        4        3
                 0.1875    0.375   -0.1875    0.375   6
     k1 = EI
                  0.375      1.00    -0.375      0.5    7
              D                                  T
                -0.1875   -0.375   0.1875   -0.375  4
                  0.375       0.5     -0.375     1.00   3


  For member  2  ,

    12EI   12EI               6EI   6EI
         =      = 0.44444EI        =     = 0.66667EI      L3       33                  L2      32

    4EI   4EI                2EI   2EI
        =     = 1.33333EI         =     = 0.66667EI    L      3               L      3

                   4         2         5         1
                 0.44444    0.66667   -0.44444   0.66667   4
     k2 = EI
                 0.66667    1.33333   -0.66667   0.66667   2
              D                                        T
                -0.44444   -0.66667   0.44444   -0.66667  5
                 0.66667    0.66667   -0.66667   1.33333   1


  Known Nodal Loads and Deflections. The nodal loads acting on the
  unconstrained degree of freedom (code number 1, 2, 3, and 4) are
  shown in Fig. a and b.


             0   1
                                  0  5
           -9  2
    Qk = D    T    and  Dk = C 0 S 6             0   3
                                  0  7
           -18 4





                                                  529

```

</details>


## PDF Page 531

![PDF page 531](chapter_15_assets/page_531.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–8.  Continued




  Load-Displacement Relation. The structure stiffness matrix is a 7 * 7 matirx
  since the highest code number is 7.Applying Q = KD ,


                    1         2        3        4         5         6        7
     0            1.33333    0.66667      0      0.66667   -0.66667     0        0    1  D1
    -9           0.66667    1.33333      0      0.66667   -0.66667     0        0    2  D2
     0              0         0        1.00     -0.375       0        0.375       0.5    3  D3
  G -18 W = EI G 0.66667    0.66667   -0.375   0.63194   -0.44444   -0.1875   -0.375 W 4 G D4 W
    Q5          -0.66667   -0.66667     0     -0.44444   0.44444      0        0    5   0
    Q6             0         0       0.375    -0.1875      0       0.1875    0.375   6   0
    Q7             0         0         0.5     -0.375       0        0.375      1.00   7   0

  From the matrix partition, Qk = K11Du + K12Dk,

       0 = EI(1.33333D1 + 0.66667D2 + 0.66667D4)                            (1)
     -9 = EI(0.66667D1 + 1.33333D2 + 0.66667D4)                            (2)
       0 = EI(D3 - 0.375D4)                                                    (3)
    -18 = EI(0.66667D1 + 0.66667D2 - 0.375D3 + 0.63194D4)                (4)


  Solving Eqs. (1) to (4),

        111.167          97.667            120             320
  D1 =          D2 =          D3 = -        D4 = -        EI          EI           EI           EI


  Using these result and applying Qu = K21Du + K22Dk

                                                 97.667                 -320  Q5 = -0.66667EIa111.167 b + a-0.66667EIb a       b + (-0.44444EI)a     b + 0 = 3.00 kN                 EI                  EI                 EI

                 120                    320
  Q6 = 0.375EIa -    b + (-0.1875EI) a -    b + 0 = 15.00 kN              EI                EI

               120                  320  Q7 = 0.5EIa -    b + (-0.375EI)a -    b + 0 = 60.00 kN  # m             EI               EI


  Superposition of these results with the (FEM),
  R5 = 3.00 + 4.50 = 7.50 kN c                                         Ans.
  R6 = 15.00 + 0 = 15.0 kN c                                           Ans.
  R7 = 60.00 + 0 = 60.0 kN  # m a                                       Ans.





                                                 530

```

</details>


## PDF Page 532

![PDF page 532](chapter_15_assets/page_532.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–9.  Determine the moments at  2 and  3  . EI is constant.                                     4 kN/m
  Assume  1  ,  2  , and  3  are rollers and  4  is pinned.                             1          2              3             4


                                                                                1      1        2     2        3    3    4


                                                                                     12 m           12 m           12 m




  The FEMs are shown on the figure.


            -19.2                D1
            -19.2                D2
    Qk = D     T        Dk = D   T
               19.2                D3
               19.2                D4


                 0.3333   0.16667
      k1 = EI c                             d                0.16667   0.3333

                 0.3333   0.16667
      k2 = EI c                             d                0.16667   0.3333

                 0.3333   0.16667
      k3 = EI c                             d                0.16667   0.3333

   K =  k1 +  k2 +  k3

                 0.3333   0.16667     0        0
                0.16667   0.6667   0.16667     0
   K = EI D                                  T                  0      0.16667   0.6667   0.16667
                  0        0      0.16667   0.3333

   Q = KD
       -19.2          0.3333   0.16667     0        0     D1
       -19.2          0.16667   0.6667   0.16667     0     D2
     D     T = EI D                                  T D   T
        19.2            0      0.16667   0.6667   0.16667   D3
        19.2            0        0      0.16667   0.3333   D4

     -19.2 = EI[0.3333D1 + 0.16667D2]
     -19.2 = EI[0.16667D1 + 0.6667D2 + 0.16667D3]
        19.2 = EI[0.16667D2 + 0.6667D3 + 0.16667D4]
        19.2 = EI[0.16667D3 + 0.16667D4]

   Solving,

    D1 =  -46.08>EI
    D2 =  -23.04>EI
    D3 = 23.04>EI
    D4 = 46.08>EI
      q = k1D





                                                  531

```

</details>


## PDF Page 533

![PDF page 533](chapter_15_assets/page_533.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–9.  Continued


      cq1 d = EI c 0.3333   0.16667 d  c -46.08>EI d
      q2         0.16667   0.3333   -23.04>EI
     q1 = EI[0.3333(-46.08>EI) + 0.16667(-23.04>EI)]
     q1 = -19.2 kN  # m
     q2 = EI[0.16667(-46.08>EI) + 0.3333(-23.04>EI)]
     q2 = -15.36 kN  # m
  Since the opposite FEM =  19.2 kN  # m is at node 1, then
   M1 = M4 = 19.2 - 19.2 = 0
  Since the FEM =  -28.8 kN  # m is at node 2, then
   M2 = M3 = -28.8 - 15.36 = 44.2 kN  # m                            Ans.





  15–10.  Determine the reactions at the supports.Assume  2                              4              5       3 k/ft  6
   is pinned and  1 and  3  are rollers. EI is constant.

                                                                                              1               2              3
                                                                                       1        1     2         2    3


                                                                                     4 ft        8 ft            8 ft        4 ft
 Member 1

              0.1875     0.75   -0.1875    0.75
      EI    0.75      4      -0.75     2
  k1 =     D                                T
        8   -0.1875   -0.75   0.1875   -0.75
                0.75      2     - 0.75     4




                                                 532

```

</details>


## PDF Page 534

![PDF page 534](chapter_15_assets/page_534.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–10.  Continued





  Member 2

                 0.1875     0.75   -0.1875    0.75
         EI    0.75      4      -0.75     2
     k2 =     D                                T
           8   -0.1875   -0.75   0.1875   -0.75
                   0.75      2      -0.75     4


   Q = KD


           8.0               4      2      0       0.75      -0.75      0     D1
          0                2      8      2       0.75       0       -0.75    D2
       - 8.0     EI    0      2      4       0        0.75      -0.75    D3
     F         V =     F                                                 V F   V
      Q4 - 24.0      8     0.75     0.75     0      0.1875   -0.1875     0       0
      Q5 - 24.0          -0.75    0      0.75   -0.1875    0.375    -0.1875    0
      Q6 - 24.0            0     -0.75   -0.75     0      -0.1875   0.1875     0


          EI
        8.0 =    [4D1 + 2D2]              8

          EI
        0 =    [2D1 + 8D2 + 2D3]             8

          EI
     -8.0 =    [2D2 + 4D3]              8


  Solving:

             16.0                             16.0
    D1 =          ,    D2 = 0 ,    D3 = -         EI                     EI

             EI        16.0
    Q4 - 24.0 =    (0.75)a    b + 0 + 0                 8      EI

    Q4 =  25.5 k                                                      Ans.

             EI          16.0       EI          16.0
    Q5 - 24.0 =    (-0.75)a    b + 0 +    (0.75)a -    b                 8        EI          8        EI

    Q5 =  21.0 k                                                      Ans.

                   EI         -16.0
    Q6 - 24.0 = 0 + 0 +    (-0.75)a      b                         8        EI

    Q6 =  25.5 k                                                      Ans.

 a + aM2 =  0;  25.5(8) - 25.5(8) = 0   (Check)
  + c aF =  0;  25.5 + 21.0 + 25.5 - 72 = 0   (Check)


                                                  533

```

</details>


## PDF Page 535

![PDF page 535](chapter_15_assets/page_535.png)

<details>
<summary>Searchable extracted text</summary>

```text

15–11.  Determine the reactions at the supports. There is a                       3                                       1
  smooth slider at  1  . EI is constant.                                                             30 kN/m

                                                                               2
                                                                                                   1                   1       2                                                                                      4
 Member Stiffness Matrix. For member  1  ,                                                       4 m
    12EI   12EI              6EI   6EI
         =      = 0.1875EI        =     = 0.375EI     L3       43                  L2      42

    4EI   4EI                2EI   2EI
        =     = EI               =     = 0.5EI    L      4               L      4

                  3        4        1        2
                 0.1875    0.375   -0.1875    0.375   3
     k1 = EI
                  0.375      1.00    -0.375      0.5    4
             D                                  T
               -0.1875   -0.375   0.1875   -0.375  1
                  0.375       0.5     -0.375     1.00   2

 Known Nodal Loads And Deflections. The nodal load acting on the unconstrained
  degree of freedom (code number 1) is shown in Fig. a.Thus,

                               0  2
    Qk = [-60] 1  and  Dk = C 0 S 3
                               0  4

  Load-Displacement Relation. The structure stiffness matrix is a 4 * 4 matirx
  since the highest code number is 4.Applying Q = KD ,

                     1        2        3        4
     -60           0.1875   -0.375   -0.1875   -0.375  1  D1
      Q2          -0.375     1.00      0.375       0.5    2   0
     D    T = EI D                                  T  D   T
      Q3          -0.1875   -0.375   0.1875    0.375   3   0
      Q4          -0.375      0.5      0.375      1.00   4   0

 From the matrix partition, Qk = K11Du + K12Dk ,

                                320
    -60 = 0.1875EID1   D1 = -                         EI

  Using this result, and applying Qu = K21Du + K22Dk ,
    Q2 = -0.375EIa -320 b + 0 = 120 kN  # m                 EI

                      320
    Q3 = -0.1875EIa-    b + 0 = 60 kN                 EI

                     320    Q4 = -0.375EIa-    b + 0 = 120 kN  # m                 EI

  Superposition these results with the FEM shown in Fig. b,
  R2 = 120 - 40 = 80 kN  # m d                                         Ans.
  R3 = 60 + 60 = 120 kN c                                             Ans.
  R4 = 120 + 40 = 160 kN  # m d                                        Ans.





                                                 534

```

</details>
