using System;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;

//TODO: This is the explicit implementation for 2D problems. Use an interface to allow for other implementations.
namespace MGroup.XFEM.Cracks.Geometry
{
    public class TipCoordinateSystem
    {
        private readonly Vector localCoordinatesOfGlobalOrigin;

        public double RotationAngle { get; }

        public Matrix RotationMatrixGlobalToLocal { get; }
        public Matrix TransposeRotationMatrixGlobalToLocal { get; } // cache this for efficiency

        /// <summary>
        /// det(J_globToLoc) = det(Q) = (cosa)^2 + (sina)^2 = 1
        /// </summary>
        public double DeterminantOfJacobianGlobalToLocalCartesian { get; }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="tipCoordinates">Coordinates of the crack tip in the global cartesian system.</param>
        /// <param name="tipRotationAngle">Counter-clockwise angle from the O-x axis of the global cartesian system to  
        ///     the T-x1 axis of the local corrdinate system of the tip (T being the tip point)</param>
        public TipCoordinateSystem(double[] tipCoordinates, double tipRotationAngle)
        {
            this.RotationAngle = tipRotationAngle;

            double cosa = Math.Cos(tipRotationAngle);
            double sina = Math.Sin(tipRotationAngle);
            RotationMatrixGlobalToLocal = Matrix.CreateFromArray(new double[,] { { cosa, sina }, { -sina, cosa } });
            TransposeRotationMatrixGlobalToLocal = RotationMatrixGlobalToLocal.Transpose();
            localCoordinatesOfGlobalOrigin = -1 * (RotationMatrixGlobalToLocal * Vector.CreateFromArray(tipCoordinates));
            DeterminantOfJacobianGlobalToLocalCartesian = 1.0; // det = (cosa)^2 +(sina)^2 = 1
        }

        public TipJacobians CalcJacobiansAt(double[] polarCoords)
        {
            return new TipJacobians(polarCoords, RotationMatrixGlobalToLocal);
        }

        public double[] MapPointGlobalCartesianToLocalCartesian(double[] cartesianGlobalPoint)
        {
            Vector local = RotationMatrixGlobalToLocal * Vector.CreateFromArray(cartesianGlobalPoint);
            local.AddIntoThis(localCoordinatesOfGlobalOrigin);
            return new double[] { local[0], local[1] };
        }

        public double[] MapPointLocalCartesianToLocalPolar(double[] cartesianLocalPoint)
        {
            double x1 = cartesianLocalPoint[0];
            double x2 = cartesianLocalPoint[1];
            double r = Math.Sqrt(x1 * x1 + x2 * x2);
            double theta = Math.Atan2(x2, x1);
            return new double[] { r, theta };
        }

        public double[] MapPointGlobalCartesianToLocalPolar(double[] cartesianGlobalPoint)
        {
            Vector local = RotationMatrixGlobalToLocal * Vector.CreateFromArray(cartesianGlobalPoint);
            local.AddIntoThis(localCoordinatesOfGlobalOrigin);
            double x1 = local[0];
            double x2 = local[1];
            double r = Math.Sqrt(x1 * x1 + x2 * x2);
            double theta = Math.Atan2(x2, x1);
            return new double[] { r, theta };
        }

        public Vector TransformScalarFieldDerivativesGlobalCartesianToLocalCartesian(Vector gradient)
        {
            return gradient * TransposeRotationMatrixGlobalToLocal;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="gradient">A 2x2 matrix, for which: Row i is the gradient of the ith component of the vector 
        ///     field, thus:    gradient = [Fx,x Fx,y; Fy,x Fy,y],
        ///     where Fi,j is the derivative of component i w.r.t. coordinate j</param>
        /// <returns></returns>
        public Matrix TransformVectorFieldDerivativesGlobalCartesianToLocalCartesian(Matrix gradient)
        {
            return RotationMatrixGlobalToLocal * (gradient * TransposeRotationMatrixGlobalToLocal);
        }

        public Tensor2D TransformTensorGlobalCartesianToLocalCartesian(Tensor2D tensor)
        {
            return tensor.Rotate(RotationAngle);
        }
    }
}
