# Linear Algebra

Linear algebra decompositions and solvers for constraint solving, least-squares fitting, and general linear system applications.

## Implemented Components

### ✅ LU Decomposition (`lu_decomposition.hpp`)
- **Doolittle's method** with partial pivoting for numerical stability
- Combined storage of L and U matrices with permutation vector
- Determinant computation from LU decomposition
- Forward/back substitution for solving Ly=Pb and Ux=y

**Key features:**
- Partial pivoting to minimize error propagation
- Singularity detection with configurable tolerance
- Efficient solve for Ax=b using LU factors
- Sign tracking for determinant calculation

### ✅ QR Decomposition (`qr_decomposition.hpp`)
- **Modified Gram-Schmidt orthogonalization** for improved numerical stability
- Orthogonal matrix Q with orthonormal columns
- Upper triangular matrix R
- Rank-deficiency detection

**Key features:**
- More stable than classical Gram-Schmidt
- Better for ill-conditioned systems
- Automatic re-orthogonalization
- Singularity detection

### ✅ Linear System Solver (`solver.hpp`)
- **Solve Ax=b**: Choose between LU and QR methods
- **Matrix inverse**: A⁻¹ computed via LU decomposition
- **Determinant**: Computed efficiently from LU factors
- Method selection for different numerical properties

**Methods:**
- `solve(A, b, SolveMethod::LU)` - Fast, general purpose
- `solve(A, b, SolveMethod::QR)` - Stable for ill-conditioned systems
- `inverse(A)` - Matrix inversion
- `determinant(A)` - Determinant computation

## Planned Components

**Decompositions:**
- SVD (Singular Value Decomposition) - for pseudoinverse, rank computation
- Cholesky decomposition - for positive definite systems, covariance matrices

**Utilities:**
- Rank computation
- Pseudoinverse (Moore-Penrose)
- Eigenvalue/eigenvector decomposition
- Condition number estimation

**Algorithms:**
- Iterative solvers (conjugate gradient, GMRES)
- Sparse matrix operations
- Matrix factorization caching

## Applications

### Physics Constraint Solving
- Solve constraint forces: **J**ᵀ*λ = **c** where **c** is constraint violations
- Iterative refinement for better conditioning

### Least-Squares Fitting
- Fit curves/planes to data: minimize ||Ax - b||²
- Normal equations: AᵀAx = Aᵀb

### Principal Component Analysis
- Covariance matrix decomposition for dimensionality reduction
- Feature extraction from high-dimensional data

### Geometric Computations
- Ray-polygon intersection
- Plane fitting
- Orthonormalization of basis vectors

### Numerical Stability Checks
- Condition number analysis
- Singularity detection
- Tolerance-based validation

### Numerical Methods
- Discretized PDEs (heat equation, wave equation)
- Finite element method assembly
- Eigenvalue problem preconditioners

## Numerical Properties

### LU Decomposition
- **Time complexity**: O(N³)
- **Space complexity**: O(N²)
- **Stability**: Good with partial pivoting
- **When to use**: General systems, constraint solving, known well-conditioned matrices

### QR Decomposition
- **Time complexity**: O(N³) modified Gram-Schmidt
- **Space complexity**: O(N²)
- **Stability**: Better for ill-conditioned systems
- **When to use**: Ill-conditioned systems, least-squares, orthonormalization

## Usage Example

```cpp
#include <core/math/linear_algebra/solver.hpp>

using phynity::math::linear_algebra::solve;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;

// Solve 3x3 system: Ax = b
MatN<3, 3, float> A = /* ... */;
VecN<3, float> b = /* ... */;

// Fast solve using LU decomposition
VecN<3, float> x = solve(A, b, SolveMethod::LU);

// Stable solve for ill-conditioned systems
VecN<3, float> x_stable = solve(A, b, SolveMethod::QR);

// Compute inverse
MatN<3, 3, float> A_inv = inverse(A);

// Compute determinant
float det = determinant(A);
```

## Planned Enhancements

- **SVD** (Singular Value Decomposition) for rank computation and pseudoinverse
- **Cholesky decomposition** for symmetric positive-definite systems (common in physics)
- **Sparse matrix solvers** for large systems with many zeros
- **Iterative refinement** for improved accuracy
- **Eigenvalue/eigenvector** computation for principal stresses and frequencies

## Status

✅ Core implementation complete and tested
