using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Interpolation.Inverse;

namespace MGroup.XFEM.Interpolation
{
	/// <summary>
	/// Isoparametric interpolation of a wedge finite element with 6 nodes. Linear shape functions.
	/// Implements singleton pattern.
	/// Authors: Dimitris Tsapetis
	/// </summary>
	public class InterpolationWedge6 : IsoparametricInterpolationBase
	{
		private static readonly InterpolationWedge6 uniqueInstance= new InterpolationWedge6();

		private InterpolationWedge6() : base(3, CellType.Wedge6, 6)
		{
			NodalNaturalCoordinates = new double[][]
			{
				new double[] { -1, 1, 0 },
				new double[] { -1, 0, 1 },
				new double[] { -1, 0, 0 },
				new double[] { 1, 1, 0 },
				new double[] { 1, 0, 1 },
				new double[] { 1, 0, 0 },
			};
		}

		/// <summary>
		/// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order
		/// of these nodes matches the order of the shape functions and is always the same for each element.
		/// </summary>
		public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

		/// <summary>
		/// Get the unique instance <see cref="InterpolationWedge6"/> object for the whole program. Thread safe.
		/// </summary>
		public static InterpolationWedge6 UniqueInstance => uniqueInstance;

		/// <summary>
		/// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
		/// </summary>
		public override void CheckElementNodes(IReadOnlyList<INode> nodes)
		{
			if (nodes.Count != 6) throw new ArgumentException(
				$"A Wedge6 finite element has 6 nodes, but {nodes.Count} nodes were provided.");
			// TODO: Also check the order of the nodes too and perhaps even the shape
		}

		/// <summary>
		/// The reverse mapping for this interpolation, namely from global cartesian coordinates to natural (element local) coordinate system.
		/// </summary>
		/// <param name="node">The nodes of the finite element in the global cartesian coordinate system.</param>
		/// <returns></returns>
		public override IInverseInterpolation CreateInverseMappingFor(IReadOnlyList<INode> node)
			=> throw new NotImplementedException("Implementation pending");

		protected sealed override double[] EvaluateAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];
			var values = new double[6];

			values[0] = 0.5 * eta * (1 - xi);
			values[1] = 0.5 * zeta * (1 - xi);
			values[2] = 0.5 * (1 - eta - zeta) * (1 - xi);
			values[3] = 0.5 * eta * (xi + 1);
			values[4] = 0.5 * zeta * (xi + 1);
			values[5] = 0.5*(1-eta-zeta)*(xi+1); 

			return values;
		}

		// Evaluation based on: https://www.code-aster.org/V2/doc/v11/en/man_r/r3/r3.01.01.pdf
		protected sealed override Matrix EvaluateGradientsAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];

			var derivatives = Matrix.CreateZero(6, 3);

			derivatives[0, 0] = -eta / 2;
			derivatives[1, 0] = -zeta / 2;
			derivatives[2, 0] = (eta  + zeta  - 1) / 2;
			derivatives[3, 0] = eta / 2;
			derivatives[4, 0] = zeta / 2;
			derivatives[5, 0] =  (1 - zeta  - eta) / 2;

			derivatives[0, 1] = (1- xi) / 2;
			derivatives[1, 1] = 0.0;
			derivatives[2, 1] = (xi - 1) / 2;
			derivatives[3, 1] = (xi + 1) / 2;
			derivatives[4, 1] = 0.0;
			derivatives[5, 1] = (-xi - 1) / 2;

			derivatives[0, 2] = 0.0;
			derivatives[1, 2] = (1 - xi) / 2;
			derivatives[2, 2] = (xi - 1) / 2;
			derivatives[3, 2] = 0.0;
			derivatives[4, 2] = (xi + 1) / 2;
			derivatives[5, 2] = (-xi - 1) / 2;

			return derivatives;
		}
	}
}
