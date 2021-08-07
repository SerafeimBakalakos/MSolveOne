using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

// Tri3 nodes:
// 2
// |  \
// 0 -- 1

//TODO: See https://www.colorado.edu/engineering/CAS/courses.d/IFEM.d/IFEM.Ch24.d/IFEM.Ch24.pdf for optimizations
namespace MGroup.XFEM.Interpolation
{
    /// <summary>
    /// Isoparametric interpolation of a triangular finite element with 3 nodes. Linear shape functions. 
    /// Implements Singleton pattern.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public class InterpolationTri3 : IsoparametricInterpolationBase
    {
        private static readonly InterpolationTri3 uniqueInstance = new InterpolationTri3();

        private InterpolationTri3() : base(2, CellType.Tri3, 3)
        {
            NodalNaturalCoordinates = new double[][]
            {
                new double[] { 0.0, 0.0 },
                new double[] { 1.0, 0.0 },
                new double[] { 0.0, 1.0 }
            };
        }

        /// <summary>
        /// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order of these
        /// nodes matches the order of the shape functions and is always the same for each element.
        /// </summary>
        public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

        /// <summary>
        /// Get the unique <see cref="InterpolationTri3"/> object for the whole program. Thread safe.
        /// </summary>
        public static InterpolationTri3 UniqueInstance => uniqueInstance;

        /// <summary>
        /// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
        /// </summary>
        public override void CheckElementNodes(IReadOnlyList<INode> nodes)
        {
            if (nodes.Count != 3) throw new ArgumentException(
                $"A Tri3 finite element has 3 nodes, but {nodes.Count} nodes were provided.");
            // TODO: Also check the order of the nodes too and perhaps even the shape
        }

        /// <summary>
        /// The inverse mapping of this interpolation, namely from global cartesian to natural (element local) coordinate system.
        /// </summary>
        /// <param name="nodes">The nodes of the finite element in the global cartesian coordinate system.</param>
        public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> nodes)
            => new InverseInterpolationTri3(nodes);
        
        protected override sealed double[] EvaluateAt(double[] naturalPoint)
        {
            double xi = naturalPoint[0];
            double eta = naturalPoint[1];
            var values = new double[3];
            values[0] = 1 - xi - eta;
            values[1] = xi;
            values[2] = eta;
            return values;
        }

        protected override sealed Matrix EvaluateGradientsAt(double[] naturalPoint)
        {
            var derivatives = Matrix.CreateZero(3, 2);

            derivatives[0, 0] = -1.0;
            derivatives[1, 0] = +1.0;
            derivatives[2, 0] = 0.0;

            derivatives[0, 1] = -1.0;
            derivatives[1, 1] = 0.0;
            derivatives[2, 1] = +1.0;

            return derivatives;
        }
    }
}
