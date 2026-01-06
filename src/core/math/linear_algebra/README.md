# Linear Algebra

Linear algebra decompositions and solvers for constraint solving, least-squares fitting, and general linear system applications.

## Implemented Components

### ✅ Cholesky Decomposition (`cholesky_decomposition.hpp`)
- **Doolittle's variant** for symmetric positive-definite (SPD) matrices
- Lower triangular matrix L with A = L*L^T
- Efficient O(N³/3) computation (2x faster than LU for SPD systems)
- Positive-definiteness validation
- Determinant and inverse computation optimized for SPD matrices

**Key features:**
- Half the computational cost and memory of general decompositions
- Numerically stable without pivoting (for SPD matrices)
- Perfect for covariance matrices and constraint Hessians
- Fast O(N²) solve using forward/back substitution

### ✅ SVD (Singular Value Decomposition) (`svd_decomposition.hpp`)
- **One-sided Jacobi iteration** for robust computation on small matrices
- Orthogonal matrices U and V with orthonormal columns
- Singular values sorted in descending order
- Numerical rank detection and condition number computation
- Moore-Penrose pseudoinverse computation

**Key features:**
- Optimal accuracy for well-conditioned and ill-conditioned matrices
- Handles rank-deficient systems gracefully
- Orthogonality verified to machine precision
- Condition number tracks system conditioning

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
- **Solve Ax=b**: Choose between LU, QR, or Cholesky methods
- **Solve SPD systems**: Optimized `solve_spd()` for symmetric positive-definite
- **Least-squares solve**: Minimize ||Ax - b||² using SVD
- **Matrix inverse**: A⁻¹ computed via LU or Cholesky (for SPD)
- **Pseudoinverse**: Moore-Penrose A⁺ computed via SVD
- **Determinant**: Efficient computation via LU or Cholesky (for SPD)
- **Matrix rank**: Numerical rank detection via SVD
- **Condition number**: κ(A) from SVD singular values

**Methods:**
- `solve(A, b, SolveMethod::LU)` - Fast, general purpose
- `solve(A, b, SolveMethod::QR)` - Stable for ill-conditioned systems
- `solve_spd(A, b)` - Optimized Cholesky for SPD matrices
- `least_squares_solve(A, b)` - Least-squares via SVD
- `inverse(A)` - Matrix inversion via LU
- `inverse_spd(A)` - Optimized inversion for SPD via Cholesky
- `pseudo_inverse(A)` - Moore-Penrose pseudoinverse
- `determinant(A)` - General determinant via LU
- `determinant_spd(A)` - Optimized determinant for SPD via Cholesky
- `matrix_rank(A, tolerance)` - Numerical rank via SVD
- `condition_number(A)` - Condition number κ(A) via SVD

## Planned Components

**Decompositions:**
- Cholesky decomposition - for positive definite systems, covariance matrices
- Eigenvalue/eigenvector decomposition - principal stresses, natural frequencies

**Utilities:**
- Iterative refinement for improved accuracy
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

### Cholesky Decomposition
- **Time complexity**: O(N³/3) - ~2x faster than LU for SPD systems
- **Space complexity**: O(N²)
- **Stability**: Excellent for SPD matrices (no pivoting needed)
- **When to use**: Covariance matrices, constraint Hessians, positive-definite systems
- **Notes**: Most efficient for physics applications, fails gracefully for non-SPD

### SVD (Singular Value Decomposition)
- **Time complexity**: O(N³) one-sided Jacobi method
- **Space complexity**: O(N²)
- **Stability**: Excellent for both well-conditioned and ill-conditioned systems
- **When to use**: Computing pseudoinverse, rank, condition number, least-squares, SVD-based analyses
- **Notes**: More robust than LU/QR for singular or near-singular systems

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
#include <core/math/linear_algebra/cholesky_decomposition.hpp>
#include <core/math/linear_algebra/svd_decomposition.hpp>

using phynity::math::linear_algebra::solve;
using phynity::math::linear_algebra::solve_spd;
using phynity::math::linear_algebra::least_squares_solve;
using phynity::math::linear_algebra::CholeskyDecomposition;
using phynity::math::linear_algebra::SVDDecomposition;
using phynity::math::matrices::MatN;
using phynity::math::vectors::VecN;

// General 3x3 system: Ax = b
MatN<3, 3, float> A = /* ... */;
VecN<3, float> b = /* ... */;

// Fast solve using LU decomposition
VecN<3, float> x = solve(A, b, SolveMethod::LU);

// SPD system - optimized 2x faster using Cholesky
MatN<3, 3, float> A_spd = /* symmetric positive-definite ... */;
VecN<3, float> x_spd = solve_spd(A_spd, b);

// Least-squares solve: minimize ||Ax - b||²
VecN<3, float> x_ls = least_squares_solve(A, b);

// Cholesky decomposition for SPD analysis
CholeskyDecomposition<3, float> chol(A_spd);
if (chol.is_positive_definite) {
    // Reconstruction: A_spd ≈ L * L^T
    MatN<3, 3, float> reconstructed = chol.reconstruct();
    
    // Optimized determinant
    float det = determinant_spd(A_spd);
    
    // Optimized inverse
    MatN<3, 3, float> A_inv = inverse_spd(A_spd);
}

// SVD-based analysis
SVDDecomposition<3, float> svd(A);
int rank = svd.rank;
float condition_num = svd.condition_number;
```

## Planned Enhancements

- **Eigenvalue/eigenvector** computation for principal stresses and frequencies
- **Iterative refinement** for improved accuracy of ill-conditioned systems
- **Sparse matrix solvers** for large systems with many zeros
- **Krylov subspace methods** (Conjugate Gradient, GMRES) for large-scale problems

## Status

✅ Cholesky decomposition complete with comprehensive tests
✅ SVD implementation complete with comprehensive tests
✅ Core decompositions (LU, QR, SVD, Cholesky) complete and tested
⏳ Eigenvalue/eigenvector computation (mid-term)
⏳ Iterative solvers (long-term)
