using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.DataStructures;
using MGroup.XFEM.Integration.Quadratures;

namespace MGroup.XFEM.Interpolation.GaussPointExtrapolation
{
    /// Calculates extrapolations of scalar, vector and tensor fields from the integration points of symmetric Gauss quadrature
    /// for tetrahedra with 1 Gauss point. This can be done at any point, but utility methods for directly outputting the 
    /// extrapolated fields at the nodes of finite elements are also provided. Note that since there is only 1 Gauss point,
    /// the scalar, vector and tensor fields are constant at all points and equal to their values at the Gauss point.
    /// Implements Singleton pattern.
    /// </summary>
    public class ExtrapolationGaussTetrahedral1Point : IGaussPointExtrapolation
    {
        private static readonly ExtrapolationGaussTetrahedral1Point uniqueInstance = new ExtrapolationGaussTetrahedral1Point();

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation2D.Quadrature"/>
        /// </summary>
        public IQuadrature Quadrature { get { return TetrahedronQuadrature.Order1Point1; } }

        /// <summary>
        /// Get the unique <see cref="ExtrapolationGaussTetrahedral1Point"/> object for the whole program. Thread safe.
        /// </summary>
        public static ExtrapolationGaussTetrahedral1Point UniqueInstance { get { return uniqueInstance; } }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateScalarFromGaussPoints(IReadOnlyList{double}, double[])"/>.
        /// </summary>
        public double ExtrapolateScalarFromGaussPoints(IReadOnlyList<double> scalarsAtGaussPoints, double[] naturalPoint)
            => scalarsAtGaussPoints[0];

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateScalarFromGaussPointsToNodes(
        /// IReadOnlyList{double}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<double> ExtrapolateScalarFromGaussPointsToNodes(IReadOnlyList<double> scalarsAtGaussPoints, 
            IIsoparametricInterpolation interpolation)
        {
            var nodalScalars = new double[interpolation.NumFunctions];
            for (int i = 0; i < nodalScalars.Length; ++i) nodalScalars[i] = scalarsAtGaussPoints[0];
            return nodalScalars;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPoints(IReadOnlyList{double[]}, double[])"/>.
        /// </summary>
        public double[] ExtrapolateTensorFromGaussPoints(IReadOnlyList<double[]> tensorsAtGaussPoints, double[] naturalPoint)
            => tensorsAtGaussPoints[0];

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPoints(IReadOnlyList{Tensor2D}, double[])"/>.
        /// </summary>
        public Tensor2D ExtrapolateTensorFromGaussPoints(IReadOnlyList<Tensor2D> tensorsAtGaussPoints, double[] naturalPoint)
            => throw new NotSupportedException("Tensor classes must be work for 2D and 3D");

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPointsToNodes(
        /// IReadOnlyList{double[]}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<double[]> ExtrapolateTensorFromGaussPointsToNodes(IReadOnlyList<double[]> tensorsAtGaussPoints,
            IIsoparametricInterpolation interpolation)
        {
            var nodalTensors = new double[interpolation.NumFunctions][];
            for (int i = 0; i < nodalTensors.Length; ++i) nodalTensors[i] = tensorsAtGaussPoints[0];
            return nodalTensors;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPointsToNodes(
        /// IReadOnlyList{Tensor3D}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<Tensor2D> ExtrapolateTensorFromGaussPointsToNodes(IReadOnlyList<Tensor2D> tensorsAtGaussPoints, 
            IIsoparametricInterpolation interpolation)
            => throw new NotSupportedException("Tensor classes must be work for 2D and 3D");

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateVectorFromGaussPoints(IReadOnlyList{double[]}, double[])"/>.
        /// </summary>
        public double[] ExtrapolateVectorFromGaussPoints(IReadOnlyList<double[]> vectorsAtGaussPoints, double[] naturalPoint)
            => vectorsAtGaussPoints[0];

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateVectorFromGaussPointsToNodes(
        /// IReadOnlyList{double[]}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<double[]> ExtrapolateVectorFromGaussPointsToNodes(IReadOnlyList<double[]> vectorsAtGaussPoints, 
            IIsoparametricInterpolation interpolation)
        {
            var nodalVectors = new double[interpolation.NumFunctions][];
            for (int i = 0; i < nodalVectors.Length; ++i) nodalVectors[i] = vectorsAtGaussPoints[0];
            return nodalVectors;
        }
    }
}
