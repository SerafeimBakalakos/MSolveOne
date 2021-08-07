using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
    public class InterpolationShell8Cohesive: IsoparametricInterpolationBase
    {
        private static readonly InterpolationShell8Cohesive uniqueInstance = new InterpolationShell8Cohesive();

        private readonly Dictionary<IQuadrature, IReadOnlyList<Matrix>> cachedN3AtGPs;

        private InterpolationShell8Cohesive() : base(2, CellType.Quad8,8)
        {
            cachedN3AtGPs = new Dictionary<IQuadrature, IReadOnlyList<Matrix>>();
            NodalNaturalCoordinates = new double[][]
            {
                //TODO: validate this
                new double[] { 1, 1 },
                new double[] { -1, 1 },
                new double[] { -1, -1 },
                new double[] { 1, -1 },
                new double[] { 0, 1 },
                new double[] { -1, 0 },
                new double[] { 0, -1 },
                new double[] { 1, 0 }
            };
        }
        public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

        public static InterpolationShell8Cohesive UniqueInstance => uniqueInstance;

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
        /// </summary>
        public override void CheckElementNodes(IReadOnlyList<INode> nodes)
        {
            if (nodes.Count != 8) throw new ArgumentException(
                $"A Shell8 finite element has 8 nodes, but {nodes.Count} nodes were provided.");
            // TODO: Also check the order of the nodes too and perhaps even the shape
        }

        public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> nodes)
            => throw new NotImplementedException();

        protected override double[] EvaluateAt(double[] naturalPoint)
        {
            double xi = naturalPoint[0];
            double eta = naturalPoint[1];
            double[] N1gp = new double[8]; //8=nShapeFunctions;

            N1gp[4] = 0.5 * (1 - Math.Pow(xi, 2)) * (1 + eta);
            N1gp[5] = 0.5 * (1 - Math.Pow(eta, 2)) * (1 - xi);
            N1gp[6] = 0.5 * (1 - Math.Pow(xi, 2)) * (1 - eta);
            N1gp[7] = 0.5 * (1 - Math.Pow(eta, 2)) * (1 + xi);

            N1gp[0] = 0.25 * (1 + xi) * (1 + eta) - 0.5 * N1gp[4] - 0.5 * N1gp[7];
            N1gp[1] = 0.25 * (1 - xi) * (1 + eta) - 0.5 * N1gp[4] - 0.5 * N1gp[5];
            N1gp[2] = 0.25 * (1 - xi) * (1 - eta) - 0.5 * N1gp[5] - 0.5 * N1gp[6];
            N1gp[3] = 0.25 * (1 + xi) * (1 - eta) - 0.5 * N1gp[6] - 0.5 * N1gp[7];

            return N1gp;
        }

        protected override Matrix EvaluateGradientsAt(double[] naturalPoint)
        {
            double xi = naturalPoint[0];
            double eta = naturalPoint[1];
            var shapeFunctionDerivativesGp = Matrix.CreateZero(2, 8); //notation per each dimension 2(0:denotes derivative ksi 1:denotes derivative heta) 8(number of shape functions and hence nodes)

            shapeFunctionDerivativesGp[0, 4] = (-xi) * (1 + eta);
            shapeFunctionDerivativesGp[0, 5] = -0.5 * (1 - Math.Pow(eta, 2));
            shapeFunctionDerivativesGp[0, 6] = 0.5 * (-2 * xi) * (1 - eta);
            shapeFunctionDerivativesGp[0, 7] = 0.5 * (1 - Math.Pow(eta, 2));
            shapeFunctionDerivativesGp[0, 0] = +0.25 * (1 + eta) - 0.5 * shapeFunctionDerivativesGp[0, 4] - 0.5 * shapeFunctionDerivativesGp[0, 7];
            shapeFunctionDerivativesGp[0, 1] = -0.25 * (1 + eta) - 0.5 * shapeFunctionDerivativesGp[0, 4] - 0.5 * shapeFunctionDerivativesGp[0, 5];
            shapeFunctionDerivativesGp[0, 2] = -0.25 * (1 - eta) - 0.5 * shapeFunctionDerivativesGp[0, 5] - 0.5 * shapeFunctionDerivativesGp[0, 6];
            shapeFunctionDerivativesGp[0, 3] = +0.25 * (1 - eta) - 0.5 * shapeFunctionDerivativesGp[0, 6] - 0.5 * shapeFunctionDerivativesGp[0, 7];

            shapeFunctionDerivativesGp[1, 4] = 0.5 * (1 - Math.Pow(xi, 2));
            shapeFunctionDerivativesGp[1, 5] = 0.5 * (-2 * eta) * (1 - xi);
            shapeFunctionDerivativesGp[1, 6] = 0.5 * (1 - Math.Pow(xi, 2)) * (-1);
            shapeFunctionDerivativesGp[1, 7] = 0.5 * (-2 * eta) * (1 + xi);
            shapeFunctionDerivativesGp[1, 0] = +0.25 * (1 + xi) - 0.5 * shapeFunctionDerivativesGp[1, 4] - 0.5 * shapeFunctionDerivativesGp[1, 7];
            shapeFunctionDerivativesGp[1, 1] = +0.25 * (1 - xi) - 0.5 * shapeFunctionDerivativesGp[1, 4] - 0.5 * shapeFunctionDerivativesGp[1, 5];
            shapeFunctionDerivativesGp[1, 2] = -0.25 * (1 - xi) - 0.5 * shapeFunctionDerivativesGp[1, 5] - 0.5 * shapeFunctionDerivativesGp[1, 6];
            shapeFunctionDerivativesGp[1, 3] = -0.25 * (1 + xi) - 0.5 * shapeFunctionDerivativesGp[1, 6] - 0.5 * shapeFunctionDerivativesGp[1, 7];

           return shapeFunctionDerivativesGp;
        }

        public IReadOnlyList<Matrix> EvaluateN3ShapeFunctionsReorganized(IQuadrature quadrature)
        {
            bool isCached = cachedN3AtGPs.TryGetValue(quadrature,
                out IReadOnlyList<Matrix> N3AtGPs);
            if (isCached) return N3AtGPs;
            else
            {
                IReadOnlyList<double[]> N1 = EvaluateFunctionsAtGaussPoints(quadrature);
                N3AtGPs = GetN3ShapeFunctionsReorganized(quadrature, N1);
                cachedN3AtGPs.Add(quadrature, N3AtGPs);
                return N3AtGPs;
            }
        }

        private IReadOnlyList<Matrix> GetN3ShapeFunctionsReorganized(IQuadrature quadrature, IReadOnlyList<double[]> N1)
        {
            //TODO reorganize cohesive shell  to use only N1 (not reorganised)

            int nGaussPoints = quadrature.IntegrationPoints.Count;
            var N3 = new Matrix[nGaussPoints]; // shapeFunctionsgpData
            for (int npoint = 0; npoint < nGaussPoints; npoint++)
            {
                double ksi = quadrature.IntegrationPoints[npoint].Coordinates[0];
                double heta = quadrature.IntegrationPoints[npoint].Coordinates[1];
                var N3gp = Matrix.CreateZero( 3, 24); //8=nShapeFunctions;

                for (int l = 0; l < 3; l++)
                {
                    for (int m = 0; m < 8; m++) N3gp[l, l + 3 * m] = N1[npoint][m];
                }
                N3[npoint] = N3gp;
            }
            return N3;
        }
    }
}
