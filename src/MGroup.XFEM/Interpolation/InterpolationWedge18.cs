using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
	/// <summary>
	/// Isoparametric interpolation of a wedge with 18 nodes. Quadratic shape functions.
	/// Implements singleton pattern.
	/// </summary>
	public class InterpolationWedge18 : IsoparametricInterpolationBase
	{
		private static readonly InterpolationWedge18 uniqueInstance= new InterpolationWedge18();

		private InterpolationWedge18() : base(3, CellType.Wedge18, 18)
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
				new double[] { 1, 0, 0.5 },
				new double[] { 0,0.5,0.5 },
				new double[] { 0,0.5,0 },
				new double[] { 0,0,0.5 }, 
			};
		}

		/// <summary>
		/// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order
		/// of these nodes matches the order of the shape functions and is always the same for each element.
		/// </summary>
		public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

		/// <summary>
		/// Get the unique instance <see cref="InterpolationWedge18"/> object for the whole program. Thread safe.
		/// </summary>
		public static InterpolationWedge18 UniqueInstance => uniqueInstance;

		/// <summary>
		/// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
		/// </summary>
		public override void CheckElementNodes(IReadOnlyList<INode> nodes)
		{
			if (nodes.Count != 18) throw new ArgumentException(
				$"A Wedge18 finite element has 18 nodes, but {nodes.Count} nodes were provided.");
			// TODO: Also check the order of the nodes too and perhaps even the shape
		}

		/// <summary>
		/// The reverse mapping for this interpolation, namely from global cartesian coordinates to natural (element local) coordinate system.
		/// </summary>
		/// <param name="node">The nodes of the finite element in the global cartesian coordinate system.</param>
		/// <returns></returns>
		public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> node) 
			=> throw new NotImplementedException("Iterative procedure needed");

		protected sealed override double[] EvaluateAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];

			var values = new double[18];

			values[0] = xi * eta * (xi - 1) * (2 * eta - 1) / 2;
			values[1] = xi * zeta * (xi - 1) * (2 * zeta - 1) / 2;
			values[2] = xi * (xi - 1) * (zeta + eta - 1) * (2 * zeta + 2 * eta - 1) / 2;
			values[3] = xi * eta * (xi + 1) * (2 * eta - 1) / 2;
			values[4] = xi * zeta * (xi + 1) * (2 * zeta - 1) / 2;
			values[5] = xi * (xi + 1) * (zeta + eta - 1) * (2 * zeta + 2 * eta - 1) / 2;
			values[6] = 2 * xi * eta * zeta * (xi - 1);
			values[7] = -2 * xi * eta * (xi - 1) * (zeta + eta - 1);
			values[8] = eta * (1 - xi * xi) * (2 * eta - 1);
			values[9] = -2 * xi * zeta * (xi - 1) * (zeta + eta - 1);
			values[10] = zeta * (1 - xi * xi) * (2 * zeta  - 1);
			values[11] = (1 - xi * xi) * (zeta + eta - 1) * (2 * zeta + 2 * eta - 1);
			values[12] = 2 * xi * eta * zeta * (xi + 1);
			values[13] = -2 * xi * eta * (xi + 1) * (zeta + eta - 1);
			values[14] = -2 * xi * zeta * (xi + 1) * (zeta + eta - 1);
			values[15] = 4 * eta * zeta * (1 - xi * xi);
			values[16] = 4 * eta * (xi * xi - 1) * (zeta + eta - 1);
			values[17] = 4 * zeta * (xi * xi - 1) * (zeta + eta - 1);

			return values;
		}

		protected sealed override Matrix EvaluateGradientsAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];

			var derivatives = Matrix.CreateZero(18, 3);

			derivatives[0, 0] = (xi * eta * (2 * eta - 1)) / 2 + (eta * (2 * eta - 1) * (xi - 1)) / 2;
			derivatives[1, 0] = (xi * zeta * (2 * zeta - 1)) / 2 + (zeta * (2 * zeta - 1) * (xi - 1)) / 2;
			derivatives[2, 0] = (xi * (2 * eta + 2 * zeta - 1) * (eta + zeta - 1)) / 2 + ((xi - 1) * (2 * eta + 2 * zeta - 1) * (eta + zeta - 1)) / 2;
			derivatives[3, 0] = (xi * eta * (2 * eta - 1)) / 2 + (eta * (2 * eta - 1) * (xi + 1)) / 2;
			derivatives[4, 0] = (xi * zeta * (2 * zeta - 1)) / 2 + (zeta * (2 * zeta - 1) * (xi + 1)) / 2;
			derivatives[5, 0] = (xi * (2 * eta + 2 * zeta - 1) * (eta + zeta - 1)) / 2 + ((xi + 1) * (2 * eta + 2 * zeta - 1) * (eta + zeta - 1)) / 2;
			derivatives[6, 0] = 2 * eta * zeta * (xi - 1) + 2 * xi * eta * zeta;
			derivatives[7, 0] = -2 * eta * (xi - 1) * (eta + zeta - 1) - 2 * xi * eta * (eta + zeta - 1);
			derivatives[8, 0] = -2 * xi * eta * (2 * eta - 1);
			derivatives[9, 0] = -2 * zeta * (xi - 1) * (eta + zeta - 1) - 2 * xi * zeta * (eta + zeta - 1);
			derivatives[10, 0] = -2 * xi * zeta * (2 * zeta - 1);
			derivatives[11, 0] = -2 * xi * (2 * eta + 2 * zeta - 1) * (eta + zeta - 1);
			derivatives[12, 0] = 2 * eta * zeta * (xi + 1) + 2 * xi * eta * zeta;
			derivatives[13, 0] = -2 * eta * (xi + 1) * (eta + zeta - 1) - 2 * xi * eta * (eta + zeta - 1);
			derivatives[14, 0] = -2 * zeta * (xi + 1) * (eta + zeta - 1) - 2 * xi * zeta * (eta + zeta - 1);
			derivatives[15, 0] = -8 * xi * eta * zeta;
			derivatives[16, 0] = 8 * xi * eta * (eta + zeta - 1);
			derivatives[17, 0] = 8 * xi * zeta * (eta + zeta - 1);

			derivatives[0, 1] = xi * eta * (xi - 1) + (xi * (2 * eta - 1) * (xi - 1)) / 2;
			derivatives[1, 1] = 0.0;
			derivatives[2, 1] = xi * (xi - 1) * (eta + zeta - 1) + (xi * (xi - 1) * (2 * eta + 2 * zeta - 1)) / 2;
			derivatives[3, 1] = xi * eta * (xi + 1) + (xi * (2 * eta - 1) * (xi + 1)) / 2;
			derivatives[4, 1] = 0.0;
			derivatives[5, 1] = xi * (xi + 1) * (eta + zeta - 1) + (xi * (xi + 1) * (2 * eta + 2 * zeta - 1)) / 2;
			derivatives[6, 1] = 2 * xi * zeta * (xi - 1);
			derivatives[7, 1] = -2 * xi * (xi - 1) * (eta + zeta - 1) - 2 * xi * eta * (xi - 1);
			derivatives[8, 1] = -2 * eta * (xi * xi - 1) - (xi * xi - 1) * (2 * eta - 1);
			derivatives[9, 1] = -2 * xi * zeta * (xi - 1);
			derivatives[10, 1] = 0.0;
			derivatives[11, 1] = -(xi * xi - 1) * (2 * eta + 2 * zeta - 1) - 2 * (xi * xi - 1) * (eta + zeta - 1);
			derivatives[12, 1] = 2 * xi * zeta * (xi + 1);
			derivatives[13, 1] = -2 * xi * (xi + 1) * (eta + zeta - 1) - 2 * xi * eta * (xi + 1);
			derivatives[14, 1] = -2 * xi * zeta * (xi + 1);
			derivatives[15, 1] = -4 * zeta * (xi * xi - 1);
			derivatives[16, 1] = 4 * eta * (xi * xi - 1) + 4 * (xi * xi - 1) * (eta + zeta - 1);
			derivatives[17, 1] = 4 * zeta * (xi * xi - 1);

			derivatives[0, 2] = 0.0;
			derivatives[1, 2] = xi * zeta * (xi - 1) + (xi * (2 * zeta - 1) * (xi - 1)) / 2;
			derivatives[2, 2] = xi * (xi - 1) * (eta + zeta - 1) + (xi * (xi - 1) * (2 * eta + 2 * zeta - 1)) / 2;
			derivatives[3, 2] = 0.0;
			derivatives[4, 2] = xi * zeta * (xi + 1) + (xi * (2 * zeta - 1) * (xi + 1)) / 2;
			derivatives[5, 2] = xi * (xi + 1) * (eta + zeta - 1) + (xi * (xi + 1) * (2 * eta + 2 * zeta - 1)) / 2;
			derivatives[6, 2] = 2 * xi * eta * (xi - 1);
			derivatives[7, 2] = -2 * xi * eta * (xi - 1);
			derivatives[8, 2] = 0.0;
			derivatives[9, 2] = -2 * xi * (xi - 1) * (eta + zeta - 1) - 2 * xi * zeta * (xi - 1);
			derivatives[10, 2] = -2 * zeta * (xi * xi - 1) - (xi * xi - 1) * (2 * zeta - 1);
			derivatives[11, 2] = -(xi * xi - 1) * (2 * eta + 2 * zeta - 1) - 2 * (xi * xi - 1) * (eta + zeta - 1);
			derivatives[12, 2] = 2 * xi * eta * (xi + 1);
			derivatives[13, 2] = -2 * xi * eta * (xi + 1);
			derivatives[14, 2] = -2 * xi * (xi + 1) * (eta + zeta - 1) - 2 * xi * zeta * (xi + 1);
			derivatives[15, 2] = -4 * eta * (xi * xi - 1);
			derivatives[16, 2] = 4 * eta * (xi * xi - 1);
			derivatives[17, 2] = 4 * zeta * (xi * xi - 1) + 4 * (xi * xi - 1) * (eta + zeta - 1);

			return derivatives;
		}
	}
}
