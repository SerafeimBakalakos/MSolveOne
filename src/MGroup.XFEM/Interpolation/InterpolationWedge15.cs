using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
	/// <summary>
	/// Isoparamteric interpolation of a wedge finite element with 15 nodes. Quadratic shape functions.
	/// Implements singleton pattern.
	/// </summary>
	public class InterpolationWedge15 : IsoparametricInterpolationBase
	{
		private static readonly InterpolationWedge15 uniqueInstance = new InterpolationWedge15();

		private InterpolationWedge15() : base(3, CellType.Wedge15, 15)
		{
			NodalNaturalCoordinates = new double[][]
			{
				new double[] { -1, 1, 0 },
				new double[] { -1, 0, 1 },
				new double[] { -1, 0, 0 },
				new double[] { 1, 1, 0 },
				new double[] { 1, 0, 1 },
				new double[] { 1, 0, 0 },

				new double[] { -1, 0.5, 0.5 },
				new double[] { -1, 0.5, 0 },
				new double[] { 0, 1, 0 },
				new double[] { -1, 0, 0.5 },

				new double[] { 0, 0, 1 },
				new double[] { 0, 0, 0 },
				new double[] { 1, 0.5, 0.5 },
				new double[] { 1, 0.5, 0 },
				new double[] { 1, 0, 0.5 }
			};
		}

		/// <summary>
		/// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order
		/// of these nodes matches the order of the shape functions and is always the same for each element.
		/// </summary>
		public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

		/// <summary>
		/// Get the unique instance <see cref="InterpolationWedge15"/> object for the whole program. Thread safe.
		/// </summary>
		public static InterpolationWedge15 UniqueInstance => uniqueInstance;

		/// <summary>
		/// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
		/// </summary>
		public override void CheckElementNodes(IReadOnlyList<INode> nodes)
		{
			if (nodes.Count != 15) throw new ArgumentException(
				$"A Wedge15 finite element has 15 nodes, but {nodes.Count} nodes were provided.");
			// TODO: Also check the order of the nodes too and perhaps even the shape
		}

		/// <summary>
		/// The reverse mapping for this interpolation, namely from global cartesian coordinates to natural (element local) coordinate system.
		/// </summary>
		/// <param name="node">The nodes of the finite element in the global cartesian coordinate system.</param>
		/// <returns></returns>
		public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> node) =>
			throw new NotImplementedException("Iterative procedure needed");

		// Evaluated according to https://www.code-aster.org/V2/doc/v11/en/man_r/r3/r3.01.01.pdf
		protected sealed override double[] EvaluateAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];

			var values = new double[15];
			values[0] = eta * (1 - xi) * (2 * eta - 2 - xi) / 2;
			values[1] = zeta * (1 - xi) * (2 * zeta - 2 - xi) / 2;
			values[2] = (xi - 1) * (1 - eta - zeta) * (xi + 2 * eta + 2 * zeta) / 2;
			values[3] = eta * (1 + xi) * (2 * eta - 2 + xi) / 2;
			values[4] = zeta * (1 + xi) * (2 * zeta - 2 + xi) / 2;
			values[5] = (-xi - 1) * (1 - eta - zeta) * (-xi + 2 * eta + 2 * zeta) / 2;
			values[6] = 2 * eta * zeta * (1 - xi);
			values[7] = 2 * eta * (1 - eta - zeta) * (1 - xi);
			values[8] = eta * (1 - xi * xi);
			values[9] = 2 * zeta * (1 - eta - zeta) * (1 - xi);
			values[10] = zeta * (1 - xi * xi);
			values[11] = (1 - eta - zeta) * (1 - xi * xi);
			values[12] = 2 * eta * zeta * (1 + xi);
			values[13] = 2 * eta * (1 - eta - zeta) * (1 + xi);
			values[14] = 2 * zeta * (1 - eta - zeta) * (1 + xi);

			return values;
		}

		protected sealed override Matrix EvaluateGradientsAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];

			var derivatives = Matrix.CreateZero(15, 3);

			derivatives[0, 0] = (eta * (xi - 2 * eta + 2)) / 2 + (eta * (xi - 1)) / 2;
			derivatives[1, 0] = (zeta * (xi - 2 * zeta + 2)) / 2 + (zeta * (xi - 1)) / 2;
			derivatives[2, 0] = -((eta + zeta - 1) * (xi + 2 * eta + 2 * zeta)) / 2 - ((xi - 1) * (eta + zeta - 1)) / 2;
			derivatives[3, 0] = (eta * (xi + 2 * eta - 2)) / 2 + (eta * (xi + 1)) / 2;
			derivatives[4, 0] = (zeta * (xi + 2 * zeta - 2)) / 2 + (zeta * (xi + 1)) / 2;
			derivatives[5, 0] = ((eta + zeta - 1) * (2 * eta - xi + 2 * zeta)) / 2 - ((xi + 1) * (eta + zeta - 1)) / 2;
			derivatives[6, 0] = -2 * eta * zeta;
			derivatives[7, 0] = 2 * eta * (eta + zeta - 1);
			derivatives[8, 0] = -2 * xi * eta;
			derivatives[9, 0] = 2 * zeta * (eta + zeta - 1);
			derivatives[10, 0] = -2 * xi * zeta;
			derivatives[11, 0] = 2 * xi * (eta + zeta - 1);
			derivatives[12, 0] = 2 * eta * zeta;
			derivatives[13, 0] = -2 * eta * (eta + zeta - 1);
			derivatives[14, 0] = -2 * zeta * (eta + zeta - 1);

			derivatives[0, 1] = ((xi - 1) * (xi - 2 * eta + 2)) / 2 - eta * (xi - 1);
			derivatives[1, 1] = 0.0;
			derivatives[2, 1] = -(xi - 1) * (eta + zeta - 1) - ((xi - 1) * (xi + 2 * eta + 2 * zeta)) / 2;
			derivatives[3, 1] = eta * (xi + 1) + ((xi + 1) * (xi + 2 * eta - 2)) / 2;
			derivatives[4, 1] = 0.0;
			derivatives[5, 1] = ((xi + 1) * (2 * eta - xi + 2 * zeta)) / 2 + (xi + 1) * (eta + zeta - 1);
			derivatives[6, 1] = -2 * zeta * (xi - 1);
			derivatives[7, 1] = 2 * (xi - 1) * (eta + zeta - 1) + 2 * eta * (xi - 1);
			derivatives[8, 1] = 1 - xi * xi;
			derivatives[9, 1] = 2 * zeta * (xi - 1);
			derivatives[10, 1] = 0.0;
			derivatives[11, 1] = xi * xi - 1;
			derivatives[12, 1] = 2 * zeta * (xi + 1);
			derivatives[13, 1] = -2 * (xi + 1) * (eta + zeta - 1) - 2 * eta * (xi + 1);
			derivatives[14, 1] = -2 * zeta * (xi + 1);

			derivatives[0, 2] = 0.0;
			derivatives[1, 2] = ((xi - 1) * (xi - 2 * zeta + 2)) / 2 - zeta * (xi - 1);
			derivatives[2, 2] = -(xi - 1) * (eta + zeta - 1) - ((xi - 1) * (xi + 2 * eta + 2 * zeta)) / 2;
			derivatives[3, 2] = 0.0;
			derivatives[4, 2] = zeta * (xi + 1) + ((xi + 1) * (xi + 2 * zeta - 2)) / 2;
			derivatives[5, 2] = ((xi + 1) * (2 * eta - xi + 2 * zeta)) / 2 + (xi + 1) * (eta + zeta - 1);
			derivatives[6, 2] = -2 * eta * (xi - 1);
			derivatives[7, 2] = 2 * eta * (xi - 1);
			derivatives[8, 2] = 0.0;
			derivatives[9, 2] = 2 * (xi - 1) * (eta + zeta - 1) + 2 * zeta * (xi - 1);
			derivatives[10, 2] = 1 - xi * xi;
			derivatives[11, 2] = xi * xi - 1;
			derivatives[12, 2] = 2 * eta * (xi + 1);
			derivatives[13, 2] = -2 * eta * (xi + 1);
			derivatives[14, 2] = -2 * (xi + 1) * (eta + zeta - 1) - 2 * zeta * (xi + 1);

			return derivatives;
		}
	}
}
