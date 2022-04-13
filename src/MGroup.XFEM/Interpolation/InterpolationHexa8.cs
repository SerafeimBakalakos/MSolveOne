using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
    /// <summary>
    /// Isoparametric interpolation of a hexahedral finite element with 8 nodes. Linear shape functions.
    /// Implements singleton pattern.
    /// </summary>
    public class InterpolationHexa8 : IsoparametricInterpolationBase
    {
        private const double oneOverEight = 0.125;

        private static readonly InterpolationHexa8 uniqueInstance= new InterpolationHexa8();

        private InterpolationHexa8() : base(3, CellType.Hexa8, 8)
        {
            NodalNaturalCoordinates = new double[][]
            {
                new double[]{-1, -1, -1},
                new double[]{1, -1, -1},
                new double[]{1, 1, -1},
                new double[]{-1, 1, -1},
                new double[]{-1, -1, 1},
                new double[]{1, -1, 1},
                new double[]{1, 1, 1},
                new double[]{-1, 1, 1},
            };
        }

        /// <summary>
        /// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order of these
        /// nodes matches the order of the shape functions and is always the same for each element.
        /// </summary>
        public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

        /// <summary>
        /// Get the unique <see cref="InterpolationHexa8"/> object for the whole program. Thread safe.
        /// </summary>
        public static InterpolationHexa8 UniqueInstance => uniqueInstance;

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.CheckElementINodes(IReadOnlyList{INode})"/>
        /// </summary>
        public override void CheckElementNodes(IReadOnlyList<INode> nodes)
        {
            if (nodes.Count != 8) throw new ArgumentException(
                $"A Hexa8 finite element has 8 nodes, but {nodes.Count} nodes were provided.");
            // TODO: Also check the order of the nodes too and perhaps even the shape
        }

        /// <summary>
        /// The inverse mapping of this interpolation, namely from global cartesian to natural (element local) coordinate system.
        /// </summary>
        /// <param name="node">The nodes of the finite element in the global cartesian coordinate system.</param>
        /// <returns></returns>
        public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> nodes) =>
            new InverseInterpolationHexa8(nodes);

        protected  override double[] EvaluateAt(double[] naturalPoint)
        {
            double xi = naturalPoint[0];
            double eta = naturalPoint[1];
            double zeta = naturalPoint[2];

            var values = new double[8];
            values[0] = oneOverEight * (1 - xi) * (1 - eta) * (1 - zeta);
            values[1] = oneOverEight * (1 + xi) * (1 - eta) * (1 - zeta);
            values[2] = oneOverEight * (1 + xi) * (1 + eta) * (1 - zeta);
            values[3] = oneOverEight * (1 - xi) * (1 + eta) * (1 - zeta);
            values[4] = oneOverEight * (1 - xi) * (1 - eta) * (1 + zeta);
            values[5] = oneOverEight * (1 + xi) * (1 - eta) * (1 + zeta);
            values[6] = oneOverEight * (1 + xi) * (1 + eta) * (1 + zeta);
            values[7] = oneOverEight * (1 - xi) * (1 + eta) * (1 + zeta);
            return values;
        }

        protected sealed override Matrix EvaluateGradientsAt(double[] naturalPoint)
        {
            double xi = naturalPoint[0];
            double eta = naturalPoint[1];
            double zeta = naturalPoint[2];
            var derivatives = Matrix.CreateZero(8, 3);

            derivatives[0, 0] = -oneOverEight * (1 - eta) * (1 - zeta);
            derivatives[1, 0] = +oneOverEight * (1 - eta) * (1 - zeta);
            derivatives[2, 0] = +oneOverEight * (1 + eta) * (1 - zeta);
            derivatives[3, 0] = -oneOverEight * (1 + eta) * (1 - zeta);
            derivatives[4, 0] = -oneOverEight * (1 - eta) * (1 + zeta);
            derivatives[5, 0] = +oneOverEight * (1 - eta) * (1 + zeta);
            derivatives[6, 0] = +oneOverEight * (1 + eta) * (1 + zeta);
            derivatives[7, 0] = -oneOverEight * (1 + eta) * (1 + zeta);

            derivatives[0, 1] = -oneOverEight * (1 - xi) * (1 - zeta);
            derivatives[1, 1] = -oneOverEight * (1 + xi) * (1 - zeta);
            derivatives[2, 1] = +oneOverEight * (1 + xi) * (1 - zeta);
            derivatives[3, 1] = +oneOverEight * (1 - xi) * (1 - zeta);
            derivatives[4, 1] = -oneOverEight * (1 - xi) * (1 + zeta);
            derivatives[5, 1] = -oneOverEight * (1 + xi) * (1 + zeta);
            derivatives[6, 1] = +oneOverEight * (1 + xi) * (1 + zeta);
            derivatives[7, 1] = +oneOverEight * (1 - xi) * (1 + zeta);

            derivatives[0, 2] = -oneOverEight * (1 - xi) * (1 - eta);
            derivatives[1, 2] = -oneOverEight * (1 + xi) * (1 - eta);
            derivatives[2, 2] = -oneOverEight * (1 + xi) * (1 + eta);
            derivatives[3, 2] = -oneOverEight * (1 - xi) * (1 + eta);
            derivatives[4, 2] = +oneOverEight * (1 - xi) * (1 - eta);
            derivatives[5, 2] = +oneOverEight * (1 + xi) * (1 - eta);
            derivatives[6, 2] = +oneOverEight * (1 + xi) * (1 + eta);
            derivatives[7, 2] = +oneOverEight * (1 - xi) * (1 + eta);

            #region untested
            //var x = xi;
            //var y = eta;
            //var z = zeta;

            //derivatives[0, 0] = -(y - 1) * (z - 1) / 8;
            //derivatives[1, 0] = (y - 1) * (z - 1) / 8;
            //derivatives[2, 0] = -(y + 1) * (z - 1) / 8;
            //derivatives[3, 0] = (y + 1) * (z - 1) / 8;
            //derivatives[4, 0] = (y - 1) * (z + 1) / 8;
            //derivatives[5, 0] = -(y - 1) * (z + 1) / 8;
            //derivatives[6, 0] = (y + 1) * (z + 1) / 8;
            //derivatives[7, 0] = -(y + 1) * (z + 1) / 8;

            //derivatives[0, 1] = -(x - 1) * (z - 1) / 8;
            //derivatives[1, 1] = (x + 1) * (z - 1) / 8;
            //derivatives[2, 1] = -(x + 1) * (z - 1) / 8;
            //derivatives[3, 1] = (x - 1) * (z - 1) / 8;
            //derivatives[4, 1] = (x - 1) * (z + 1) / 8;
            //derivatives[5, 1] = -(x + 1) * (z + 1) / 8;
            //derivatives[6, 1] = (x + 1) * (z + 1) / 8;
            //derivatives[7, 1] = -(x - 1) * (z + 1) / 8;

            //derivatives[0, 2] = -(x - 1) * (y - 1) / 8;
            //derivatives[1, 2] = (x + 1) * (y - 1) / 8;
            //derivatives[2, 2] = -(x + 1) * (y + 1) / 8;
            //derivatives[3, 2] = (x - 1) * (y + 1) / 8;
            //derivatives[4, 2] = (x - 1) * (y - 1) / 8;
            //derivatives[5, 2] = -(x + 1) * (y - 1) / 8;
            //derivatives[6, 2] = (x + 1) * (y + 1) / 8;
            //derivatives[7, 2] = -(x - 1) * (y + 1) / 8;
            #endregion

            return derivatives;
        }
    }
}
