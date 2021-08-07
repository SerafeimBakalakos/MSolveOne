using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
	/// <summary>
	/// Isoparametric interpolation of pyramid finite element with 5 nodes. Linear shape functions.
	/// Implements singleton pattern.
	/// Authors: Dimitris Tsapetis
	/// </summary>
	public class InterpolationPyra5 : IsoparametricInterpolationBase
	{
		private static  readonly InterpolationPyra5 uniqueInstance= new InterpolationPyra5();

		private InterpolationPyra5() : base(3, CellType.Pyra5, 5)
		{
			NodalNaturalCoordinates= new double[][]
			{
				new double[] { 1,0,0 },
				new double[] { 0,1,0 },
				new double[] { -1,0,0 },
				new double[] { 0,-1,0 },
				new double[] { 0,0,1 }
			};
		}

		/// <summary>
		/// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order
		/// of these nodes matches the order of the shape functions and is always the same for each element.
		/// </summary>
		public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }
		
		/// <summary>
		/// Get the unique instance <see cref="InterpolationPyra5"/> object for the whole program. Thread safe.
		/// </summary>
		public static InterpolationPyra5 UniqueInstance => uniqueInstance;

		/// <summary>
		/// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
		/// </summary>
		public override void CheckElementNodes(IReadOnlyList<INode> nodes)
		{
			if (nodes.Count != 5) throw new ArgumentException(
				$"A Pyra5 finite element has 5 nodes, but {nodes.Count} nodes were provided.");
			// TODO: Also check the order of the nodes too and perhaps even the shape
		}

		/// <summary>
		/// The reverse mapping for this interpolation, namely from global cartesian coordinates to natural (element local) coordinate system.
		/// </summary>
		/// <param name="node">The nodes of the finite element in the global cartesian coordinate system.</param>
		/// <returns></returns>
		public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> node)
			=> throw new NotImplementedException();

		protected sealed override double[] EvaluateAt(double[] naturalPoint)
		{
			var values = new double[5];
			var xi = naturalPoint[0];
			var eta = naturalPoint[1];
			var zeta = naturalPoint[2];

			values[0] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (-xi + eta + zeta - 1) * (-xi - eta + zeta - 1) / (4 * (1 - zeta));
			values[1] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (-xi - eta + zeta - 1) * (xi - eta + zeta - 1) / (4 * (1 - zeta));
			values[2] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (xi + eta + zeta - 1) * (xi - eta + zeta - 1) / (4 * (1 - zeta));
			values[3] = (Math.Abs(zeta - 1) < 10e-10) ? 0 : (xi + eta + zeta - 1) * (-xi + eta + zeta - 1) / (4 * (1 - zeta));
			values[4] = zeta;

			return values;
		}

		protected override Matrix EvaluateGradientsAt(double[] naturalPoint)
		{
			var xi = naturalPoint[0];
			var eta = naturalPoint[1];
			var zeta = naturalPoint[2];

			var derivatives = Matrix.CreateZero(5, 3);
			derivatives[0, 0] = -(xi + eta - zeta + 1) / (4 * zeta - 4) - (xi - eta - zeta + 1) / (4 * zeta - 4);
			derivatives[1, 0] = (xi + eta - zeta + 1) / (4 * zeta - 4) + (xi - eta + zeta - 1) / (4 * zeta - 4);
			derivatives[2, 0] = -(xi - eta + zeta - 1) / (4 * zeta - 4) - (xi + eta + zeta - 1) / (4 * zeta - 4);
			derivatives[3, 0] = (xi - eta - zeta + 1) / (4 * zeta - 4) + (xi + eta + zeta - 1) / (4 * zeta - 4);
			derivatives[4, 0] = 0.0;

			derivatives[0, 1] = (xi + eta - zeta + 1) / (4 * zeta - 4) - (xi - eta - zeta + 1) / (4 * zeta - 4);
			derivatives[1, 1] = (xi - eta + zeta - 1) / (4 * zeta - 4) - (xi + eta - zeta + 1) / (4 * zeta - 4);
			derivatives[2, 1] = (xi + eta + zeta - 1) / (4 * zeta - 4) - (xi - eta + zeta - 1) / (4 * zeta - 4);
			derivatives[3, 1] = (xi - eta - zeta + 1) / (4 * zeta - 4) - (xi + eta + zeta - 1) / (4 * zeta - 4);
			derivatives[4, 1] = 0.0;

			derivatives[0, 2] = (xi + eta - zeta + 1) / (4 * zeta - 4) + (xi - eta - zeta + 1) / (4 * zeta - 4) +
								(4 * (xi + eta - zeta + 1) * (xi - eta - zeta + 1)) / Math.Pow(4 * zeta - 4, 2);
			derivatives[1, 2] = (xi + eta - zeta + 1) / (4 * zeta - 4) - (xi - eta + zeta - 1) / (4 * zeta - 4) -
								(4 * (xi + eta - zeta + 1) * (xi - eta + zeta - 1)) / Math.Pow(4 * zeta - 4, 2);
			derivatives[2, 2] = (4 * (xi - eta + zeta - 1) * (xi + eta + zeta - 1)) / Math.Pow(4 * zeta - 4, 2) -
								(xi + eta + zeta - 1) / (4 * zeta - 4) - (xi - eta + zeta - 1) / (4 * zeta - 4);
			derivatives[3, 2] = (xi - eta - zeta + 1) / (4 * zeta - 4) - (xi + eta + zeta - 1) / (4 * zeta - 4) -
								(4 * (xi - eta - zeta + 1) * (xi + eta + zeta - 1)) / Math.Pow(4 * zeta - 4, 2);
			derivatives[4, 2] = 1.0;

			return derivatives;
		}
	}
}
