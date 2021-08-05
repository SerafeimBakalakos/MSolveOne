using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
	/// <summary>
	/// Isoparametric interpolation of a pyramid finite element with 13 nodes. Quadratic shape functions.
	/// Implements singleton pattern.
	/// Authors: Dimitris Tsapetis
	/// </summary>
	public class InterpolationPyra13 : IsoparametricInterpolationBase
	{
		private static readonly InterpolationPyra13 uniqueInstance=new InterpolationPyra13();

		private InterpolationPyra13() : base(3, CellType.Pyra13, 13)
		{
			NodalNaturalCoordinates = new double[][]
			{
				new double[] { 1,0,0},
				new double[] { 0,1,0},
				new double[] { -1,0,0},
				new double[] { 0,-1,0},
				new double[] { 0,0,1},

				new double[] { 0.5,0.5,0},
				new double[] { 0.5,-0.5,0},
				new double[] { 0.5,0,0.5},
				new double[] { -0.5,0.5,0},
				new double[] { 0,0.5,0.5},

				new double[] { -0.5,-0.5,0},
				new double[] { -0.5,0,0.5},
				new double[] { 0,-0.5,0.5},
			};
		}

		/// <summary>
		/// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order
		/// of these nodes matches the order of the shape functions and is always the same for each element.
		/// </summary>
		public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

		/// <summary>
		/// Get the unique instance <see cref="InterpolationPyra13"/> object for the whole program. Thread safe.
		/// </summary>
		public static InterpolationPyra13 UniqueInstance => uniqueInstance;

		/// <summary>
		/// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
		/// </summary>
		public override void CheckElementNodes(IReadOnlyList<INode> nodes)
		{
			if (nodes.Count != 13) throw new ArgumentException(
				$"A Pyra13 finite element has 13 nodes, but {nodes.Count} nodes were provided.");
			// TODO: Also check the order of the nodes too and perhaps even the shape
		}

		/// <summary>
		/// The reverse mapping for this interpolation, namely from global cartesian coordinates to natural (element local) coordinate system.
		/// </summary>
		/// <param name="node">The nodes of the finite element in the global cartesian coordinate system.</param>
		/// <returns></returns>
		public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> node) 
			=> throw new NotImplementedException("Iterative procedure required");

		protected override double[] EvaluateAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];

			var values = new double[13];
			values[0] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (-xi + eta + zeta - 1) * (-xi - eta + zeta - 1) * (xi - 0.5) / (2 * (1 - zeta));
			values[1] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (-xi - eta + zeta - 1) * (xi - eta + zeta - 1) * (eta - 0.5) / (2 * (1 - zeta));
			values[2] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (xi - eta + zeta - 1) * (xi + eta + zeta - 1) * (-xi - 0.5) / (2 * (1 - zeta));
			values[3] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (xi + eta + zeta - 1) * (-xi + eta + zeta - 1) * (-eta - 0.5) / (2 * (1 - zeta));
			values[4] = 2 * zeta * (zeta - 0.5);
			values[5] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : -(-xi + eta + zeta - 1) * (-xi - eta + zeta - 1) * (xi - eta + zeta - 1) / (2 * (1 - zeta));
			values[6] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : -(xi + eta + zeta - 1) * (-xi + eta + zeta - 1) * (-xi - eta + zeta - 1) / (2 * (1 - zeta));
			values[7] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : zeta * (-xi + eta + zeta - 1) * (-xi - eta + zeta - 1) / (1 - zeta);
			values[8] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : -(-xi - eta + zeta - 1) * (xi - eta + zeta - 1) * (xi + eta + zeta - 1) / (2 * (1 - zeta));
			values[9] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : zeta * (-xi - eta + zeta - 1) * (xi - eta + zeta - 1) / (1 - zeta);
			values[10] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : -(xi - eta + zeta - 1) * (xi + eta + zeta - 1) * (-xi + eta + zeta - 1) / (2 * (1 - zeta));
			values[11] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : zeta * (xi - eta + zeta - 1) * (xi + eta + zeta - 1) / (1 - zeta);
			values[12] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : zeta * (xi + eta + zeta - 1) * (-xi + eta + zeta - 1) / (1 - zeta);
			return values;
		}


		// TODO: verify derivatives of Pyra13
		protected override Matrix EvaluateGradientsAt(double[] naturalPoint)
		{
			double x = naturalPoint[0];
			double y = naturalPoint[1];
			double z = naturalPoint[2];

			var derivatives = Matrix.CreateZero(13, 3);

			derivatives[0, 0] = -((x - 1 / 2.0) * (x - y - z + 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) -
								((x - 1 / 2.0) * (x + y - z + 1)) / (2 * z - 2);
			derivatives[1, 0] = ((y - 1 / 2.0) * (x + y - z + 1)) / (2 * z - 2) +
								((y - 1 / 2.0) * (x - y + z - 1)) / (2 * z - 2);
			derivatives[2, 0] = ((x + 1 / 2.0) * (x + y + z - 1)) / (2 * z - 2) +
								((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2) +
								((x + 1 / 2.0) * (x - y + z - 1)) / (2 * z - 2);
			derivatives[3, 0] = -((y + 1 / 2.0) * (x - y - z + 1)) / (2 * z - 2) -
								((y + 1 / 2.0) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[4, 0] = 0.0;
			derivatives[5, 0] = ((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2) +
								((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) +
								((x - y + z - 1) * (x - y - z + 1)) / (2 * z - 2);
			derivatives[6, 0] = ((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2) +
								((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) +
								((x + y - z + 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[7, 0] = -(z * (x + y - z + 1)) / (z - 1) - (z * (x - y - z + 1)) / (z - 1);
			derivatives[8, 0] = -((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[9, 0] = (z * (x + y - z + 1)) / (z - 1) + (z * (x - y + z - 1)) / (z - 1);
			derivatives[10, 0] = -((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								 ((x - y + z - 1) * (x - y - z + 1)) / (2 * z - 2) -
								 ((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[11, 0] = -(z * (x - y + z - 1)) / (z - 1) - (z * (x + y + z - 1)) / (z - 1);
			derivatives[12, 0] = (z * (x - y - z + 1)) / (z - 1) + (z * (x + y + z - 1)) / (z - 1);

			derivatives[0, 1] = ((x - 1 / 2.0) * (x + y - z + 1)) / (2 * z - 2) -
								((x - 1 / 2.0) * (x - y - z + 1)) / (2 * z - 2);
			derivatives[1, 1] = ((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2) -
								((y - 1 / 2.0) * (x + y - z + 1)) / (2 * z - 2) +
								((y - 1 / 2.0) * (x - y + z - 1)) / (2 * z - 2);
			derivatives[2, 1] = ((x + 1 / 2.0) * (x - y + z - 1)) / (2 * z - 2) -
								((x + 1 / 2.0) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[3, 1] = ((y + 1 / 2.0) * (x + y + z - 1)) / (2 * z - 2) -
								((y + 1 / 2.0) * (x - y - z + 1)) / (2 * z - 2) -
								((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[4, 1] = 0.0;
			derivatives[5, 1] = ((x - y + z - 1) * (x - y - z + 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2);
			derivatives[6, 1] = ((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2) +
								((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[7, 1] = (z * (x + y - z + 1)) / (z - 1) - (z * (x - y - z + 1)) / (z - 1);
			derivatives[8, 1] = ((x + y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2) -
								((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[9, 1] = (z * (x - y + z - 1)) / (z - 1) - (z * (x + y - z + 1)) / (z - 1);
			derivatives[10, 1] = ((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								 ((x - y + z - 1) * (x - y - z + 1)) / (2 * z - 2) +
								 ((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2);
			derivatives[11, 1] = (z * (x + y + z - 1)) / (z - 1) - (z * (x - y + z - 1)) / (z - 1);
			derivatives[12, 1] = (z * (x - y - z + 1)) / (z - 1) - (z * (x + y + z - 1)) / (z - 1);

			derivatives[0, 2] = ((x - 1 / 2.0) * (x - y - z + 1)) / (2 * z - 2) +
								((x - 1 / 2.0) * (x + y - z + 1)) / (2 * z - 2) +
								(2 * (x - 1 / 2.0) * (x + y - z + 1) * (x - y - z + 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[1, 2] = ((y - 1 / 2.0) * (x + y - z + 1)) / (2 * z - 2) -
								((y - 1 / 2.0) * (x - y + z - 1)) / (2 * z - 2) -
								(2 * (y - 1 / 2.0) * (x + y - z + 1) * (x - y + z - 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[2, 2] = ((x + 1 / 2.0) * (x + y + z - 1)) / (2 * z - 2) +
								((x + 1 / 2.0) * (x - y + z - 1)) / (2 * z - 2) -
								(2 * (x + 1 / 2.0) * (x - y + z - 1) * (x + y + z - 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[3, 2] = ((y + 1 / 2.0) * (x + y + z - 1)) / (2 * z - 2) -
								((y + 1 / 2.0) * (x - y - z + 1)) / (2 * z - 2) +
								(2 * (y + 1 / 2.0) * (x - y - z + 1) * (x + y + z - 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[4, 2] = 4 * z - 1;
			derivatives[5, 2] = ((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2) -
								((x - y + z - 1) * (x - y - z + 1)) / (2 * z - 2) -
								(2 * (x + y - z + 1) * (x - y + z - 1) * (x - y - z + 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[6, 2] = ((x + y - z + 1) * (x - y - z + 1)) / (2 * z - 2) -
								((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								(2 * (x + y - z + 1) * (x - y - z + 1) * (x + y + z - 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[7, 2] = (z * (x + y - z + 1)) / (z - 1) - ((x + y - z + 1) * (x - y - z + 1)) / (z - 1) +
								(z * (x - y - z + 1)) / (z - 1) +
								(z * (x + y - z + 1) * (x - y - z + 1)) / Math.Pow(z - 1, 2);
			derivatives[8, 2] = ((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x + y + z - 1)) / (2 * z - 2) -
								((x + y - z + 1) * (x - y + z - 1)) / (2 * z - 2) +
								(2 * (x + y - z + 1) * (x - y + z - 1) * (x + y + z - 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[9, 2] = (z * (x + y - z + 1)) / (z - 1) - (z * (x - y + z - 1)) / (z - 1) +
								((x + y - z + 1) * (x - y + z - 1)) / (z - 1) -
								(z * (x + y - z + 1) * (x - y + z - 1)) / Math.Pow(z - 1, 2);
			derivatives[10, 2] = ((x - y + z - 1) * (x + y + z - 1)) / (2 * z - 2) -
								 ((x - y + z - 1) * (x - y - z + 1)) / (2 * z - 2) -
								 ((x - y - z + 1) * (x + y + z - 1)) / (2 * z - 2) +
								 (2 * (x - y + z - 1) * (x - y - z + 1) * (x + y + z - 1)) / Math.Pow(2 * z - 2, 2);
			derivatives[11, 2] = (z * (x - y + z - 1) * (x + y + z - 1)) / Math.Pow(z - 1, 2) -
								 ((x - y + z - 1) * (x + y + z - 1)) / (z - 1) - (z * (x + y + z - 1)) / (z - 1) -
								 (z * (x - y + z - 1)) / (z - 1);
			derivatives[12, 2] = (z * (x - y - z + 1)) / (z - 1) - (z * (x + y + z - 1)) / (z - 1) +
								 ((x - y - z + 1) * (x + y + z - 1)) / (z - 1) -
								 (z * (x - y - z + 1) * (x + y + z - 1)) / Math.Pow(z - 1, 2);

			return derivatives;
		}
	}
}
