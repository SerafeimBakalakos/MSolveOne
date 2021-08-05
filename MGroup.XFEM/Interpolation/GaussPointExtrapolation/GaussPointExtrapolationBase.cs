using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.DataStructures;
using MGroup.XFEM.Integration.Quadratures;

//TODO: Perhaps I should use dictionaries for matching input and output at nodes or at least gauss points.
//TODO: Perhaps I should use generic methods when the same thing is done for scalar, vector, tensor fields. However that would 
//      complicate the interface only to avoid a few lines of boilerplate code.
//TODO: prevent triangular quadratures from accessing quadrilateral nodes and vice-versa
//TODO: cache the shape functions at the nodes.
namespace MGroup.XFEM.Interpolation.GaussPointExtrapolation
{
    /// <summary>
    /// Basic implementation of <see cref="IGaussPointExtrapolation"/>.
    /// </summary>
    public abstract class GaussPointExtrapolationBase : IGaussPointExtrapolation
    {
        /// <summary>
        /// Each <see cref="IIsoparametricInterpolation"/> is mapped to a 2D array that contains the values of the 
        /// extrapolation functions calculated at the nodes of the finite element using that interpolation. Each row corresponds
        /// to a different node. Each columns corresponds to a different extrapolation function.
        /// </summary>
        private Dictionary<IIsoparametricInterpolation, double[][]> cachedExtrapolationFunctionsAtNodes;

        protected GaussPointExtrapolationBase(IQuadrature quadrature)
        {
            this.cachedExtrapolationFunctionsAtNodes = new Dictionary<IIsoparametricInterpolation, double[][]>();
            this.Quadrature = quadrature;
        }

        /// <summary>
        /// The integration rule which defines the integration points used for extrapolating values and defining an auxiliary 
        /// coordinate system.
        /// </summary>
        public IQuadrature Quadrature { get; }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateScalarFromGaussPoints(IReadOnlyList{double}, double[])"/>.
        /// </summary>
        public double ExtrapolateScalarFromGaussPoints(IReadOnlyList<double> scalarsAtGaussPoints, double[] naturalPoint)
        {
            double[] extrapolationFunctions = EvaluateExtrapolationFunctionsAt(naturalPoint);
            double scalar = 0;
            for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
            {
                scalar += extrapolationFunctions[gp] * scalarsAtGaussPoints[gp];
            }
            return scalar;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateScalarFromGaussPointsToNodes(
        /// IReadOnlyList{double}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<double> ExtrapolateScalarFromGaussPointsToNodes(IReadOnlyList<double> scalarsAtGaussPoints, 
            IIsoparametricInterpolation interpolation)
        {
            double[][] nodalExtrapolationFunctions = EvaluateExtrapolationFunctionsAtNodes(interpolation);
            IReadOnlyList<double[]> nodes = interpolation.NodalNaturalCoordinates;
            var nodalScalars = new double[nodes.Count];
            for (int i = 0; i < nodes.Count; ++i)
            {
                //nodalScalars[i] = ExtrapolateScalarFromGaussPoints(scalarsAtGaussPoints, nodes[i]); // for debugging
                double scalar = 0;
                for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
                {
                    scalar += nodalExtrapolationFunctions[i][gp] * scalarsAtGaussPoints[gp];
                }
                nodalScalars[i] = scalar;
            }
            return nodalScalars;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPoints(IReadOnlyList{double[]}, double[])"/>.
        /// </summary>
        public double[] ExtrapolateTensorFromGaussPoints(IReadOnlyList<double[]> tensorsAtGaussPoints, double[] naturalPoint)
        {
            double[] extrapolationFunctions = EvaluateExtrapolationFunctionsAt(naturalPoint);
            var tensor = new double[3]; //In 2D problems, symmetric tensors have 3 entries. TODO: replace with Tensor2D class.
            for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
            {
                tensor[0] += extrapolationFunctions[gp] * tensorsAtGaussPoints[gp][0];
                tensor[1] += extrapolationFunctions[gp] * tensorsAtGaussPoints[gp][1];
                tensor[2] += extrapolationFunctions[gp] * tensorsAtGaussPoints[gp][2];
            }
            return tensor;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPoints(IReadOnlyList{Tensor2D}, double[])"/>.
        /// </summary>
        public Tensor2D ExtrapolateTensorFromGaussPoints(IReadOnlyList<Tensor2D> tensorsAtGaussPoints, double[] naturalPoint)
        {
            double[] extrapolationFunctions = EvaluateExtrapolationFunctionsAt(naturalPoint);
            var tensor = new double[3]; //In 2D problems, symmetric tensors have 3 entries. TODO: replace with Tensor2D class.
            for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
            {
                tensor[0] += extrapolationFunctions[gp] * tensorsAtGaussPoints[gp].XX;
                tensor[1] += extrapolationFunctions[gp] * tensorsAtGaussPoints[gp].YY;
                tensor[2] += extrapolationFunctions[gp] * tensorsAtGaussPoints[gp].XY;
            }
            return new Tensor2D(tensor[0], tensor[1], tensor[2]);
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPointsToNodes(
        /// IReadOnlyList{double[]}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<double[]> ExtrapolateTensorFromGaussPointsToNodes(IReadOnlyList<double[]> tensorsAtGaussPoints, 
            IIsoparametricInterpolation interpolation)
        {
            double[][] nodalExtrapolationFunctions = EvaluateExtrapolationFunctionsAtNodes(interpolation);
            IReadOnlyList<double[]> nodes = interpolation.NodalNaturalCoordinates;
            var nodalTensors = new double[nodes.Count][];
            for (int i = 0; i < nodes.Count; ++i)
            {
                //nodalTensors[i] = ExtrapolateVectorFromGaussPoints(tensorsAtGaussPoints, nodes[i]); // for debugging
                var tensor = new double[3];
                for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
                {
                    tensor[0] += nodalExtrapolationFunctions[i][gp] * tensorsAtGaussPoints[gp][0];
                    tensor[1] += nodalExtrapolationFunctions[i][gp] * tensorsAtGaussPoints[gp][1];
                    tensor[2] += nodalExtrapolationFunctions[i][gp] * tensorsAtGaussPoints[gp][2];
                }
                nodalTensors[i] = tensor;
            }
            return nodalTensors;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateTensorFromGaussPointsToNodes(
        /// IReadOnlyList{Tensor2D}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<Tensor2D> ExtrapolateTensorFromGaussPointsToNodes(IReadOnlyList<Tensor2D> tensorsAtGaussPoints,
            IIsoparametricInterpolation interpolation)
        {
            double[][] nodalExtrapolationFunctions = EvaluateExtrapolationFunctionsAtNodes(interpolation);
            IReadOnlyList<double[]> nodes = interpolation.NodalNaturalCoordinates;
            var nodalTensors = new Tensor2D[nodes.Count];
            for (int i = 0; i < nodes.Count; ++i)
            {
                //nodalTensors[i] = ExtrapolateVectorFromGaussPoints(tensorsAtGaussPoints, nodes[i]); // for debugging
                var tensor = new double[3];
                for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
                {
                    tensor[0] += nodalExtrapolationFunctions[i][gp] * tensorsAtGaussPoints[gp].XX;
                    tensor[1] += nodalExtrapolationFunctions[i][gp] * tensorsAtGaussPoints[gp].YY;
                    tensor[2] += nodalExtrapolationFunctions[i][gp] * tensorsAtGaussPoints[gp].XY;
                }
                nodalTensors[i] = new Tensor2D(tensor[0], tensor[1], tensor[2]);
            }
            return nodalTensors;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateVectorFromGaussPoints(IReadOnlyList{double[]}, double[])"/>.
        /// </summary>
        public double[] ExtrapolateVectorFromGaussPoints(IReadOnlyList<double[]> vectorsAtGaussPoints, double[] naturalPoint)
        {
            double[] extrapolationFunctions = EvaluateExtrapolationFunctionsAt(naturalPoint);
            var vector = new double[2]; //In 2D problems, vector fields have 2 entries. TODO: replace with Vector2 class.
            for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
            {
                vector[0] += extrapolationFunctions[gp] * vectorsAtGaussPoints[gp][0];
                vector[1] += extrapolationFunctions[gp] * vectorsAtGaussPoints[gp][1];
            }
            return vector;
        }

        /// <summary>
        /// See <see cref="IGaussPointExtrapolation.ExtrapolateVectorFromGaussPointsToNodes(
        /// IReadOnlyList{double[]}, IIsoparametricInterpolation)"/>
        /// </summary>
        public IReadOnlyList<double[]> ExtrapolateVectorFromGaussPointsToNodes(IReadOnlyList<double[]> vectorsAtGaussPoints, 
            IIsoparametricInterpolation interpolation)
        {
            double[][] nodalExtrapolationFunctions = EvaluateExtrapolationFunctionsAtNodes(interpolation);
            IReadOnlyList<double[]> nodes = interpolation.NodalNaturalCoordinates;
            var nodalVectors = new double[nodes.Count][];
            for (int i = 0; i < nodes.Count; ++i)
            {
                //nodalVectors[i] = ExtrapolateVectorFromGaussPoints(tensorsAtGaussPoints, nodes[i]); // for debugging
                var vector = new double[2];
                for (int gp = 0; gp < Quadrature.IntegrationPoints.Count; ++gp)
                {
                    vector[0] += nodalExtrapolationFunctions[i][gp] * vectorsAtGaussPoints[gp][0];
                    vector[1] += nodalExtrapolationFunctions[i][gp] * vectorsAtGaussPoints[gp][1];
                }
                nodalVectors[i] = vector;
            }
            return nodalVectors;
        }

        /// <summary>
        /// Calculates the functions used for extrapolating quantities from the integration points to a given point, at the 
        /// given point.
        /// </summary>
        /// <param name="point">The coordinates of the point where the extrapolation functions will be calculated, in the 
        ///     natural (element local) system.</param>
        /// <returns></returns>
        protected abstract double[] EvaluateExtrapolationFunctionsAt(double[] naturalPoint);

        private double[][] EvaluateExtrapolationFunctionsAtNodes(IIsoparametricInterpolation interpolation)
        {
            bool isCached = cachedExtrapolationFunctionsAtNodes.TryGetValue(interpolation,
                out double[][] nodalExtrapolationFunctions);
            if (!isCached)
            {
                IReadOnlyList<double[]> nodes = interpolation.NodalNaturalCoordinates;
                nodalExtrapolationFunctions = new double[nodes.Count][];
                for (int i = 0; i < nodes.Count; ++i)
                {
                    nodalExtrapolationFunctions[i] = EvaluateExtrapolationFunctionsAt(nodes[i]);
                }
                cachedExtrapolationFunctionsAtNodes.Add(interpolation, nodalExtrapolationFunctions);
            }
            return nodalExtrapolationFunctions;
        }
    }
}
