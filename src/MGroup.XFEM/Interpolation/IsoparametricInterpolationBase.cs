using System.Collections.Generic;
using System.Diagnostics;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation.Inverse;
using MGroup.XFEM.Interpolation.Jacobians;
using MGroup.XFEM.Extensions;

namespace MGroup.XFEM.Interpolation
{
    /// <summary>
    /// Basic implementation of <see cref="IIsoparametricInterpolation"/>.
    /// </summary>
    public abstract class IsoparametricInterpolationBase : IIsoparametricInterpolation
    {
        private readonly Dictionary<IQuadrature, IReadOnlyList<double[]>> cachedFunctionsAtGPs;
        private readonly Dictionary<IQuadrature, IReadOnlyList<Matrix>> cachedNaturalGradientsAtGPs;
        private readonly int dimension;

        protected IsoparametricInterpolationBase(int dimension, CellType cellType, int numFunctions)
        {
            Debug.Assert(dimension == 2 || dimension == 3);
            this.dimension = dimension;
            this.CellType = cellType;
            this.NumFunctions = numFunctions;
            this.cachedFunctionsAtGPs = new Dictionary<IQuadrature, IReadOnlyList<double[]>>();
            this.cachedNaturalGradientsAtGPs = new Dictionary<IQuadrature, IReadOnlyList<Matrix>>();
        }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.CellType"/>.
        /// </summary>
        public CellType CellType { get; }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.NumFunctions"/>.
        /// </summary>
        public int NumFunctions { get; }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.NodalNaturalCoordinates"/>.
        /// </summary>
        public abstract IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.CreateInverseMappingFor(IReadOnlyList{INode})"/>.
        /// </summary>
        public abstract IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> nodes);

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.EvaluateAllAt(IReadOnlyList{INode}, double[])"/>.
        /// </summary>
        public EvalInterpolation EvaluateAllAt(IReadOnlyList<INode> nodes, double[] naturalPoint)
        {
            var shapeFunctions = EvaluateAt(naturalPoint);
            Matrix naturalShapeDerivatives = EvaluateGradientsAt(naturalPoint);
            return new EvalInterpolation(dimension, nodes, shapeFunctions, naturalShapeDerivatives,
                new IsoparametricJacobian(dimension, nodes, naturalShapeDerivatives));
        }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.EvaluateAllAtGaussPoints(IReadOnlyList{INode}, IQuadrature)"/>.
        /// </summary>
        public IReadOnlyList<EvalInterpolation> EvaluateAllAtGaussPoints(IReadOnlyList<INode> nodes, IQuadrature quadrature)
        {
            // The shape functions and natural derivatives at each Gauss point are probably cached from previous calls
            IReadOnlyList<double[]> shapeFunctionsAtGPs = EvaluateFunctionsAtGaussPoints(quadrature);
            IReadOnlyList<Matrix> naturalShapeDerivativesAtGPs = EvaluateNaturalGradientsAtGaussPoints(quadrature);

            // Calculate the Jacobians and shape derivatives w.r.t. global cartesian coordinates at each Gauss point
            int numGPs = quadrature.IntegrationPoints.Count;
            var interpolationsAtGPs = new EvalInterpolation[numGPs];
            for (int gp = 0; gp < numGPs; ++gp)
            {
                interpolationsAtGPs[gp] = new EvalInterpolation(dimension, nodes, shapeFunctionsAtGPs[gp],
                    naturalShapeDerivativesAtGPs[gp], 
                    new IsoparametricJacobian(dimension, nodes, naturalShapeDerivativesAtGPs[gp]));
            }
            return interpolationsAtGPs;
        }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.EvaluateFunctionsAt(double[])"/>.
        /// </summary>
        public double[] EvaluateFunctionsAt(double[] naturalPoint)
            => EvaluateAt(naturalPoint);

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.EvaluateFunctionsAtGaussPoints(IQuadrature)"/>.
        /// </summary>
        public IReadOnlyList<double[]> EvaluateFunctionsAtGaussPoints(IQuadrature quadrature)
        {
            bool isCached = cachedFunctionsAtGPs.TryGetValue(quadrature,
                out IReadOnlyList<double[]> shapeFunctionsAtGPs);
            if (isCached) return shapeFunctionsAtGPs;
            else
            {
                int numGPs = quadrature.IntegrationPoints.Count;
                var shapeFunctionsAtGPsArray = new double[numGPs][];
                for (int gp = 0; gp < numGPs; ++gp)
                {
                    GaussPoint gaussPoint = quadrature.IntegrationPoints[gp];
                    shapeFunctionsAtGPsArray[gp] = EvaluateAt(gaussPoint.Coordinates);
                }
                cachedFunctionsAtGPs.Add(quadrature, shapeFunctionsAtGPsArray);
                return shapeFunctionsAtGPsArray;
            }
        }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.EvaluateNaturalGradientsAt(double[])".
        /// </summary>
        public Matrix EvaluateNaturalGradientsAt(double[] naturalPoint)
            => EvaluateGradientsAt(naturalPoint);

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.EvaluateNaturalGradientsAtGaussPoints(IQuadrature)"/>.
        /// </summary>
        /// <param name="quadrature"></param>
        public IReadOnlyList<Matrix> EvaluateNaturalGradientsAtGaussPoints(IQuadrature quadrature)
        {
            bool isCached = cachedNaturalGradientsAtGPs.TryGetValue(quadrature,
                out IReadOnlyList<Matrix> naturalGradientsAtGPs);
            if (isCached) return naturalGradientsAtGPs;
            else
            {
                int numGPs = quadrature.IntegrationPoints.Count;
                var naturalGradientsAtGPsArray = new Matrix[numGPs];
                for (int gp = 0; gp < numGPs; ++gp)
                {
                    GaussPoint gaussPoint = quadrature.IntegrationPoints[gp];
                    naturalGradientsAtGPsArray[gp] = EvaluateGradientsAt(gaussPoint.Coordinates);
                }
                cachedNaturalGradientsAtGPs.Add(quadrature, naturalGradientsAtGPsArray);
                return naturalGradientsAtGPsArray;
            }
        }

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.TransformNaturalToCartesian(IReadOnlyList{INode}, double[])"/>.
        /// </summary>
        public double[] TransformNaturalToCartesian(IReadOnlyList<INode> nodes, double[] naturalPoint)
        {
            double[] shapeFunctionValues = EvaluateAt(naturalPoint);
            var cartesianPoint = new double[dimension];
            for (int n = 0; n < nodes.Count; ++n)
            {
                double[] node = nodes[n].Coordinates();
                for (int i = 0; i < dimension; ++i)
                {
                    cartesianPoint[i] += shapeFunctionValues[n] * node[i];
                }
            }
            return cartesianPoint;
        }

        public abstract void CheckElementNodes(IReadOnlyList<INode> nodes);

        /// <summary>
        /// Evaluate shape function at a given point expressed in the natural coordinate system. Each entry corresponds to a
        /// different shape function.
        /// </summary>
        protected abstract double[] EvaluateAt(double[] naturalPoint);

        /// <summary>
        /// Evaluate derivatives of shape functions with respect to natural coordinates at a given point expressed in the 
        /// natural coordinate system. Each row corresponds to a different shape function, column 0 corresponds to derivatives
        /// with respect to Xi coordinate and column 1 corresponds to derivatives with respect to Eta coordinate.
        /// </summary>
        protected abstract Matrix EvaluateGradientsAt(double[] naturalPoint);
    }
}
