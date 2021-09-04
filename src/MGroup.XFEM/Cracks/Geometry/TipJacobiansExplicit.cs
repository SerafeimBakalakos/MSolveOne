using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.XFEM.Cracks.Geometry
{
    public class TipJacobiansExplicit
    {
        private readonly Matrix inverseJacobianPolarToLocal;
        private readonly Matrix inverseJacobianPolarToGlobal;

        public TipJacobiansExplicit(double[] pointPolarCoords, Matrix systemRotationMatrixGlobalToLocal)
        {
            double r = pointPolarCoords[0];
            double cosTheta = Math.Cos(pointPolarCoords[1]);
            double sinTheta = Math.Sin(pointPolarCoords[1]);
            inverseJacobianPolarToLocal = Matrix.CreateFromArray(new double[,] 
                { { cosTheta, sinTheta }, {-sinTheta / r , cosTheta / r } });

            inverseJacobianPolarToGlobal = inverseJacobianPolarToLocal * systemRotationMatrixGlobalToLocal;
        }

        public Vector TransformScalarFieldDerivativesLocalPolarToLocalCartesian(Vector gradient)
        {
            return gradient * inverseJacobianPolarToLocal;
        }

        public Vector TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(Vector gradient)
        {
            return gradient * inverseJacobianPolarToGlobal;
        }

        /// <summary>
        /// Attention: The input vector field is differentiated w.r.t. the polar cartesian system coordinates.
        /// The output vector field is differentiated w.r.t. the local cartesian system coordinates. However the 
        /// representations of both vector fields (aka the coordinates of the vectors) are in the local cartesian 
        /// coordinate system.
        /// </summary>
        /// <param name="gradient">
        /// A 2x2 matrix, for which: Row i is the gradient of the ith component of the vector field, thus:    
        /// gradient = [Fr,r Fr,theta; Ftheta,r Ftheta,theta],
        /// where Fi,j is the derivative of component i w.r.t. coordinate j</param>
        /// <returns></returns>
        public Matrix TransformVectorFieldDerivativesLocalPolarToLocalCartesian(Matrix gradient)
        {
            return gradient * inverseJacobianPolarToLocal;
        }
    }
}
