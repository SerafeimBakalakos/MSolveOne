using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
    public class InterpolationHexa8Reverse: IsoparametricInterpolationBase
    {
        private static readonly InterpolationHexa8Reverse uniqueInstance = new InterpolationHexa8Reverse();

        private InterpolationHexa8Reverse() : base(3, CellType.Hexa8, 8)
        {
            NodalNaturalCoordinates = new double[][]
            {
                new double[]{1, 1, 1},
                new double[]{-1, 1, 1},
                new double[]{-1, -1, 1},
                new double[]{1, -1, 1},
                new double[]{1, 1, -1},
                new double[]{-1, 1, -1},
                new double[]{-1, -1, -1},
                new double[]{1, -1, -1}
            };
        }

        public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

        public static InterpolationHexa8Reverse UniqueInstance => uniqueInstance;

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{Node})"/>
        /// </summary>
        public override void CheckElementNodes(IReadOnlyList<INode> nodes)
        {
            if (nodes.Count != 8) throw new ArgumentException(
                $"A Hexa8 finite element has 8 nodes, but {nodes.Count} nodes were provided.");
            // TODO: Also check the order of the nodes too and perhaps even the shape
        }

        public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> nodes) 
            => throw new NotImplementedException();

        protected override double[] EvaluateAt(double[] naturalPoint)
        {
            throw new NotImplementedException(); //TODO: fill these
            //var shapeFunctions = new double[8];
            //shapeFunctions[0] = ;
            //shapeFunctions[1] = ;
            //shapeFunctions[2] = ;
            //shapeFunctions[3] = ;
            //shapeFunctions[4] = ;
            //shapeFunctions[5] = ;
            //shapeFunctions[6] = ;
            //shapeFunctions[7] = ;
        }

        protected override Matrix EvaluateGradientsAt(double[] naturalPoint)
        {
            double xi = naturalPoint[0];
            double eta = naturalPoint[1];
            double zeta = naturalPoint[2];
            var naturalDerivatives = Matrix.CreateZero(3, 8);

            // Derivatives with respect to Xi
            naturalDerivatives[0, 0] = +0.125 * (1 + eta) * (1 + zeta);
            naturalDerivatives[0, 1] = -0.125 * (1 + eta) * (1 + zeta);
            naturalDerivatives[0, 2] = -0.125 * (1 - eta) * (1 + zeta);
            naturalDerivatives[0, 3] = +0.125 * (1 - eta) * (1 + zeta);
            naturalDerivatives[0, 4] = +0.125 * (1 + eta) * (1 - zeta);
            naturalDerivatives[0, 5] = -0.125 * (1 + eta) * (1 - zeta);
            naturalDerivatives[0, 6] = -0.125 * (1 - eta) * (1 - zeta);
            naturalDerivatives[0, 7] = +0.125 * (1 - eta) * (1 - zeta);

            // Derivatives with respect to Eta
            naturalDerivatives[1, 0] = 0.125 * (1 + xi) * (+1) * (1 + zeta);
            naturalDerivatives[1, 1] = 0.125 * (1 - xi) * (+1) * (1 + zeta);
            naturalDerivatives[1, 2] = 0.125 * (1 - xi) * (-1) * (1 + zeta);
            naturalDerivatives[1, 3] = 0.125 * (1 + xi) * (-1) * (1 + zeta);
            naturalDerivatives[1, 4] = 0.125 * (1 + xi) * (+1) * (1 - zeta);
            naturalDerivatives[1, 5] = 0.125 * (1 - xi) * (+1) * (1 - zeta);
            naturalDerivatives[1, 6] = 0.125 * (1 - xi) * (-1) * (1 - zeta);
            naturalDerivatives[1, 7] = 0.125 * (1 + xi) * (-1) * (1 - zeta);

            // Derivatives with respect to Zeta
            naturalDerivatives[2, 0] = 0.125 * (1 + xi) * (1 + eta) * (+1);
            naturalDerivatives[2, 1] = 0.125 * (1 - xi) * (1 + eta) * (+1);
            naturalDerivatives[2, 2] = 0.125 * (1 - xi) * (1 - eta) * (+1);
            naturalDerivatives[2, 3] = 0.125 * (1 + xi) * (1 - eta) * (+1);
            naturalDerivatives[2, 4] = 0.125 * (1 + xi) * (1 + eta) * (-1);
            naturalDerivatives[2, 5] = 0.125 * (1 - xi) * (1 + eta) * (-1);
            naturalDerivatives[2, 6] = 0.125 * (1 - xi) * (1 - eta) * (-1);
            naturalDerivatives[2, 7] = 0.125 * (1 + xi) * (1 - eta) * (-1);

            return naturalDerivatives;
        }
    }
}
