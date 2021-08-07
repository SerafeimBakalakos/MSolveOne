using System;
using System.Collections.Generic;
using MGroup.XFEM.Interpolation.Inverse;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Discretization;

namespace MGroup.XFEM.Interpolation
{
	/// <summary>
	/// Isoparametric interpolation of a hexahedral finite element with 8 nodes. Quadratic shape functions.
	/// Implements singleton pattern.
	/// </summary>
	public class InterpolationHexa20 : IsoparametricInterpolationBase
	{
		private static  readonly InterpolationHexa20 uniqueInstance= new InterpolationHexa20();

		private InterpolationHexa20() : base(3, CellType.Hexa20, 20)
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

				new double[]{0, -1, -1},
				new double[]{-1, 0, -1},
				new double[]{-1, -1, 0},
				new double[]{1, 0, -1},

				new double[]{1, -1, 0},
				new double[]{0, 1, -1},
				new double[]{1, 1, 0},
				new double[]{-1, 1, 0},

				new double[]{0, -1, 1},
				new double[]{-1, 0, 1},
				new double[]{1, 0, 1},
				new double[]{0, 1, 1},
			};
		}

		/// <summary>
		/// The coordinates of the finite element's nodes in the natural (element local) coordinate system. The order of these
		/// nodes matches the order of the shape functions and is always the same for each element.
		/// </summary>
		public override IReadOnlyList<double[]> NodalNaturalCoordinates { get; }

		/// <summary>
		/// Get the unique <see cref="InterpolationHexa20"/> object for the whole program. Thread safe.
		/// </summary>
		public static InterpolationHexa20 UniqueInstance => uniqueInstance;

		/// <summary>
		/// See <see cref="IIsoparametricInterpolation.CheckElementNodes(IReadOnlyList{INode})"/>
		/// </summary>
		public override void CheckElementNodes(IReadOnlyList<INode> nodes)
		{
			if (nodes.Count != 20) throw new ArgumentException(
				$"A Hexa20 finite element has 20 nodes, but {nodes.Count} nodes were provided.");
			// TODO: Also check the order of the nodes too and perhaps even the shape
		}

		/// <summary>
		/// The inverse mapping of this interpolation, namely from global cartesian to natural (element local) coordinate system.
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

			var values = new double[20];
			values[0] = 1 / 8.0 * (1 - xi) * (1 - eta) * (1 - zeta) * (-2 - xi - eta - zeta);
			values[1] = 1 / 8.0 * (1 + xi) * (1 - eta) * (1 - zeta) * (-2 + xi - eta - zeta);
			values[2] = 1 / 8.0 * (1 + xi) * (1 + eta) * (1 - zeta) * (-2 + xi + eta - zeta);
			values[3] = 1 / 8.0 * (1 - xi) * (1 + eta) * (1 - zeta) * (-2 - xi + eta - zeta);

			values[4] = 1 / 8.0 * (1 - xi) * (1 - eta) * (1 + zeta) * (-2 - xi - eta + zeta);
			values[5] = 1 / 8.0 * (1 + xi) * (1 - eta) * (1 + zeta) * (-2 + xi - eta + zeta); 
			values[6] = 1 / 8.0 * (1 + xi) * (1 + eta) * (1 + zeta) * (-2 + xi + eta + zeta); 
			values[7] = 1 / 8.0 * (1 - xi) * (1 + eta) * (1 + zeta) * (-2 - xi + eta + zeta);

			values[8] = 1 / 4.0 * (1 - xi * xi) * (1 - eta) * (1 - zeta);
			values[9] = 1 / 4.0 * (1 - eta * eta) * (1 - xi) * (1 - zeta);
			values[10] = 1 / 4.0 * (1 - zeta * zeta) * (1 - xi) * (1 - eta);
			values[11] = 1 / 4.0 * (1 - eta * eta) * (1 + xi) * (1 - zeta);

			values[12] = 1 / 4.0 * (1 - zeta * zeta) * (1 + xi) * (1 - eta);
			values[13] = 1 / 4.0 * (1 - xi * xi) * (1 + eta) * (1 - zeta);
			values[14] = 1 / 4.0 * (1 - zeta * zeta) * (1 + xi) * (1 + eta);
			values[15] = 1 / 4.0 * (1 - zeta * zeta) * (1 - xi) * (1 + eta);

			values[16] = 1 / 4.0 * (1 - xi * xi) * (1 - eta) * (1 + zeta);
			values[17] = 1 / 4.0 * (1 - eta * eta) * (1 - xi) * (1 + zeta);
			values[18] = 1 / 4.0 * (1 - eta * eta) * (1 + xi) * (1 + zeta);
			values[19] = 1 / 4.0 * (1 - xi * xi) * (1 + eta) * (1 + zeta);
			return values;
		}

		protected sealed override Matrix EvaluateGradientsAt(double[] naturalPoint)
		{
			var xi = naturalPoint[0];
			var eta = naturalPoint[1];
			var zeta = naturalPoint[2];

			var derivatives = Matrix.CreateZero(20, 3);

			derivatives[0, 0] = (xi - 1 )/8 * (eta - 1) * (zeta - 1) + ((eta - 1) * (zeta - 1) * (xi + eta + zeta + 2)) / 8;
			derivatives[1, 0] = (xi  + 1 )/8 * (eta - 1) * (zeta - 1) - ((eta - 1) * (zeta - 1) * (eta - xi + zeta + 2)) / 8;
			derivatives[2, 0] = -((eta + 1) * (zeta - 1) * (xi + eta - zeta - 2)) / 8 - (xi  + 1 ) /8* (eta + 1) * (zeta - 1);
			derivatives[3, 0] = -((eta + 1) * (zeta - 1) * (xi - eta + zeta + 2)) / 8 - (xi  - 1 ) /8* (eta + 1) * (zeta - 1);
			derivatives[4, 0] = -((eta - 1) * (zeta + 1) * (xi + eta - zeta + 2)) / 8 - (xi  - 1 )/8 * (eta - 1) * (zeta + 1);
			derivatives[5, 0] = -((eta - 1) * (zeta + 1) * (xi - eta + zeta - 2)) / 8 - (xi  + 1 )/8 * (eta - 1) * (zeta + 1);
			derivatives[6, 0] = (xi  + 1 )/8 * (eta + 1) * (zeta + 1) + ((eta + 1) * (zeta + 1) * (xi + eta + zeta - 2)) / 8;
			derivatives[7, 0] = (xi  - 1 )/8 * (eta + 1) * (zeta + 1) + ((eta + 1) * (zeta + 1) * (xi - eta - zeta + 2)) / 8;
			derivatives[8, 0] = -(xi * (eta - 1) * (zeta - 1)) / 2;
			derivatives[9, 0] = -(eta *eta  - 1 )/4 * (zeta - 1);
			derivatives[10, 0] = -(zeta *zeta - 1 )/4 * (eta - 1);
			derivatives[11, 0] = (eta *eta  - 1 )/4 * (zeta - 1);
			derivatives[12, 0] = (zeta *zeta  - 1 )/4 * (eta - 1);
			derivatives[13, 0] = (xi * (eta + 1) * (zeta - 1)) / 2;
			derivatives[14, 0] = -(zeta *zeta - 1 )/4 * (eta + 1);
			derivatives[15, 0] = (zeta *zeta - 1 )/4 * (eta + 1);
			derivatives[16, 0] = (xi * (eta - 1) * (zeta + 1)) / 2;
			derivatives[17, 0] = (eta *eta  - 1 )/4 * (zeta + 1);
			derivatives[18, 0] = -(eta *eta  - 1)/4 * (zeta + 1);
			derivatives[19, 0] = -(xi * (eta + 1) * (zeta + 1)) / 2;

			derivatives[0, 1] = (xi  - 1 )/8 * (zeta - 1) * (xi + eta + zeta + 2) + (xi  - 1)/8 * (eta - 1) * (zeta - 1);
			derivatives[1, 1] = -(xi  + 1 )/8 * (eta - 1) * (zeta - 1) - (xi  + 1)/8 * (zeta - 1) * (eta - xi + zeta + 2);
			derivatives[2, 1] = -(xi + 1)/8 * (eta + 1) * (zeta - 1) - (xi  + 1)/8 * (zeta - 1) * (xi + eta - zeta - 2);
			derivatives[3, 1] = (xi  - 1 )/8 * (eta + 1) * (zeta - 1) - (xi  - 1 )/8 * (zeta - 1) * (xi - eta + zeta + 2);
			derivatives[4, 1] = -(xi - 1 )/8 * (eta - 1) * (zeta + 1) - (xi  - 1 )/8 * (zeta + 1) * (xi + eta - zeta + 2);
			derivatives[5, 1] = (xi  + 1 )/8 * (eta - 1) * (zeta + 1) - (xi  + 1 )/8 * (zeta + 1) * (xi - eta + zeta - 2);
			derivatives[6, 1] = (xi  + 1 )/8 * (zeta + 1) * (xi + eta + zeta - 2) + (xi + 1 )/8 * (eta + 1) * (zeta + 1);
			derivatives[7, 1] = (xi  - 1 )/8 * (zeta + 1) * (xi - eta - zeta + 2) - (xi  - 1 )/8 * (eta + 1) * (zeta + 1);
			derivatives[8, 1] = -(xi *xi - 1 )/4 * (zeta - 1);
			derivatives[9, 1] = -(eta * (xi - 1) * (zeta - 1)) / 2;
			derivatives[10, 1] = -(zeta*zeta  - 1 )/4 * (xi - 1);
			derivatives[11, 1] = (eta * (xi + 1) * (zeta - 1)) / 2;
			derivatives[12, 1] = (zeta *zeta  - 1 )/4 * (xi + 1);
			derivatives[13, 1] = (xi *xi - 1 )/4 * (zeta - 1);
			derivatives[14, 1] = -(zeta *zeta  - 1)/4 * (xi + 1);
			derivatives[15, 1] = (zeta *zeta  - 1 )/4 * (xi - 1);
			derivatives[16, 1] = (xi *xi  - 1 )/4 * (zeta + 1);
			derivatives[17, 1] = (eta * (xi - 1) * (zeta + 1)) / 2;
			derivatives[18, 1] = -(eta * (xi + 1) * (zeta + 1)) / 2;
			derivatives[19, 1] = -(xi *xi  - 1 )/4 * (zeta + 1);

			derivatives[0, 2] = (xi  - 1 )/8 * (eta - 1) * (xi + eta + zeta + 2) + (xi  - 1 )/8 * (eta - 1) * (zeta - 1);
			derivatives[1, 2] = -(xi  + 1)/8 * (eta - 1) * (zeta - 1) - (xi  + 1 )/8 * (eta - 1) * (eta - xi + zeta + 2);
			derivatives[2, 2] = (xi  + 1 )/8 * (eta + 1) * (zeta - 1) - (xi  + 1 )/8 * (eta + 1) * (xi + eta - zeta - 2);
			derivatives[3, 2] = -(xi  - 1 )/8 * (eta + 1) * (zeta - 1) - (xi  - 1 )/8 * (eta + 1) * (xi - eta + zeta + 2);
			derivatives[4, 2] = (xi  - 1 )/8 * (eta - 1) * (zeta + 1) - (xi  - 1 ) /8* (eta - 1) * (xi + eta - zeta + 2);
			derivatives[5, 2] = -(xi  + 1 )/8 * (eta - 1) * (zeta + 1) - (xi  + 1 )/8 * (eta - 1) * (xi - eta + zeta - 2);
			derivatives[6, 2] = (xi  + 1 )/8 * (eta + 1) * (xi + eta + zeta - 2) + (xi  + 1 )/8 * (eta + 1) * (zeta + 1);
			derivatives[7, 2] = (xi  - 1 )/8 * (eta + 1) * (xi - eta - zeta + 2) - (xi - 1 )/8 * (eta + 1) * (zeta + 1);
			derivatives[8, 2] = -(xi *xi  - 1 )/4 * (eta - 1);
			derivatives[9, 2] = -(eta *eta  - 1 )/4 * (xi - 1);
			derivatives[10, 2] = -(zeta * (xi - 1) * (eta - 1)) / 2;
			derivatives[11, 2] = (eta *eta  - 1 )/4 * (xi + 1);
			derivatives[12, 2] = (zeta * (xi + 1) * (eta - 1)) / 2;
			derivatives[13, 2] = (xi *xi  - 1 )/4 * (eta + 1);
			derivatives[14, 2] = -(zeta * (xi + 1) * (eta + 1)) / 2;
			derivatives[15, 2] = (zeta * (xi - 1) * (eta + 1)) / 2;
			derivatives[16, 2] = (xi *xi  - 1 )/4 * (eta - 1);
			derivatives[17, 2] = (eta *eta  - 1 )/4 * (xi - 1);
			derivatives[18, 2] = -(eta *eta  - 1 )/4 * (xi + 1);
			derivatives[19, 2] = -(xi *xi  - 1 )/4 * (eta + 1);
			return derivatives;
		}
	}
}
