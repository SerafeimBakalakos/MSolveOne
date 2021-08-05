using System;
using System.Collections.Generic;
using System.Diagnostics;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Matrices;

//TODO: once we know that an exception will be thrown, try to pinpoint the error: wrong node order, clockwise node order, the  
//      element's shape is too distorted, midpoints are too close to corners in quadratic elements, etc...
namespace MGroup.XFEM.Interpolation.Jacobians
{
    /// <summary>
    /// This class encapsulates the determinant and inverse of the Jacobian matrix for a 3D isoparametric mapping.
    /// Let f be a mapping: x \in R^2 -> f(x) \in R^2 or x \in R^3 -> f(x) \in R^3. The Jacobian matrix of the mapping is 
    /// (in numerator layout): J = [df_1/dx_1 df_1/dx_2; df_2/dx_1 df_2/dx_2] or 
    /// J = [df_1/dx_1 df_1/dx_2 df_1/dx_3; df_2/dx_1 df_2/dx_2 df_2/dx_3; df_3/dx_1 df_3/dx_2 df_3/dx_3]. 
    /// Note that some sources call the transpose of this matrix as J. In FEM we are usually interested in the determinant and 
    /// inverse of the Jacobian matrix.
    /// </summary>
    public class IsoparametricJacobian
    {
        private const double determinantTolerance = 1E-8; // This needs to be in a static settings class.

        private readonly int dimension;

        /// <summary>
        /// The caller (usually the interpolation class) assumes responsibility for matching the nodes to the shape function 
        /// derivatives.
        /// </summary>
        /// <param name="nodes">The nodes used for the interpolation.</param>
        /// <param name="naturalDerivatives">The shape function derivatives at a specific integration point.</param>
        public IsoparametricJacobian(int dimension, IReadOnlyList<INode> nodes, Matrix naturalDerivatives)
        {
            Debug.Assert(dimension == 2 || dimension == 3);
            this.dimension = dimension;

            // The original matrix is not stored. Only the inverse and the determinant
            DirectMatrix = CalculateJacobianMatrix(dimension, nodes, naturalDerivatives);
            (InverseMatrix, DirectDeterminant) = DirectMatrix.InvertAndDeterminant();
            //(InverseMatrix, DirectDeterminant) = InvertAndDeterminant(DirectMatrix);
            if (DirectDeterminant < determinantTolerance)
            {
                throw new ArgumentException("Jacobian determinant is negative or under the allowed tolerance"
                    + $" ({DirectDeterminant} < {determinantTolerance}). Check the order of nodes or the element geometry.");
            }
        }

        /// <summary>
        /// The determinant of the direct Jacobian matrix <see cref="DirectMatrix"/>.
        /// </summary>
        public double DirectDeterminant { get; }

        /// <summary>
        /// The Jacobian matrix of the direct mapping. Numerator layout is used:
        /// J = [df_1/dx_1 df_1/dx_2; df_2/dx_1 df_2/dx_2] or 
        /// J = [df_1/dx_1 df_1/dx_2 df_1/dx_3; df_2/dx_1 df_2/dx_2 df_2/dx_3; df_3/dx_1 df_3/dx_2 df_3/dx_3].
        /// </summary>
        public Matrix DirectMatrix { get; }

        /// <summary>
        /// The inverse of the Jacobian matrix. Numerator layout used is used:
        /// inv(J) = [dx_1/df_1 dx_1/df_2 ; dx_2/df_1 dx_2/df_2] or 
        /// inv(J) = [dx_1/df_1 dx_1/df_2 dx_1/df_3; dx_2/df_1 dx_2/df_2 dx_2/df_3; dx_3/df_1 dx_3/df_2 dx_3/df_3]
        /// </summary>
        public Matrix InverseMatrix { get; }

        /// <summary>
        /// Transforms the gradient of a vector-valued function from the natural to the global cartesian coordinate system.
        /// </summary>
        /// <param name="naturalGradient">The gradient of a vector-valued function in the natural coordinate system. Each row 
        ///     corresponds to the gradient of a single component of the vector function. Each column corresponds to the 
        ///     derivatives of all components with respect to a single coordinate.</param>
        public Matrix TransformNaturalDerivativesToCartesian(Matrix naturalGradient)
            => InverseMatrix.MultiplyLeft(naturalGradient);

        /// <summary>
        /// Transforms the gradient of a scalar-valued function from the natural to the global cartesian coordinate system.
        /// </summary>
        /// <param name="naturalGradient">The gradient of a scalar-valued function in the natural coordinate system. Each entry 
        ///     corresponds to the derivative with respect to a single coordinate.</param>
        public double[] TransformNaturalDerivativesToCartesian(double[] naturalGradient)
        {
            // naturalGradient * inverseJ = 1-by-2 * 2-by-2 = 1-by-2 or
            // naturalGradient * inverseJ = 1-by-3 * 3-by-3 = 1-by-3
            var result = new double[dimension];
            for (int i = 0; i < dimension; ++i)
            {
                for (int j = 0; j < dimension; ++j)
                {
                    result[i] += InverseMatrix[j, i] * naturalGradient[j];
                }
            }
            return result;
        }

        private static Matrix CalculateJacobianMatrix(int dimension, IReadOnlyList<INode> nodes, Matrix naturalDerivatives)
        {
            //TODO: describe this as a matrix operation
            var jacobianMatrix = Matrix.CreateZero(dimension, dimension);
            for (int n = 0; n < nodes.Count; ++n)
            {
                double[] x = nodes[n].Coordinates();
                for (int j = 0; j < dimension; ++j)
                {
                    for (int i = 0; i < dimension; ++i)
                    {
                        jacobianMatrix[i, j] += naturalDerivatives[n, j] * x[i];
                    }
                }
            }
            return jacobianMatrix;
        }
    }
}
