using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.Mesh
{
	/// <summary>
	/// This works only for a very specific division of 1 Quad9 into 4 Tri3.
	/// </summary>
	public class DualCartesianSimplicialSymmetricMesh2D : DualCartesianSimplicialSymmetricMeshBase
	{
		private DualCartesianSimplicialSymmetricMesh2D(UniformCartesianMesh2D coarseMesh, UniformSimplicialSymmetricMesh2D fineMesh)
			: base(2, coarseMesh, fineMesh, 4)
		{
		}

		public override IIsoparametricInterpolation CoarseElementInterpolation { get; } = InterpolationQuad4.UniqueInstance;

		public override IIsoparametricInterpolation FineElementInterpolation { get; } = InterpolationTri3.UniqueInstance;

		public override DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
		{
			// Find in which subtriangle this point lies.
			double xi = coarseNaturalCoords[0];
			double eta = coarseNaturalCoords[1];
			int subtriangle;
			if (eta <= xi && eta <= -xi)
			{
				subtriangle = 0;
			}
			else if (eta <= xi && eta > -xi)
			{
				subtriangle = 1;
			}
			else if (eta > xi && eta > -xi)
			{
				subtriangle = 2;
			}
			else
			{
				Debug.Assert(eta > xi && eta <= -xi);
				subtriangle = 3;
			}

			// Map from the coarse quad to the fine triangle.
			var coordsFineTriangle = new double[2];
			if (subtriangle == 0)
			{
				//   /\
				//  /  \
				// /____\
				coordsFineTriangle[0] = 0.5 * (-xi - eta);
				coordsFineTriangle[1] = 0.5 * (+xi - eta);
			}
			else if (subtriangle == 1)
			{
				//   /|
				//  / |
				//  \ |
				//   \|
				coordsFineTriangle[0] = 0.5 * (+xi - eta);
				coordsFineTriangle[1] = 0.5 * (+xi + eta);
			}
			else if (subtriangle == 2)
			{
				// ____
				// \  /
				//  \/
				coordsFineTriangle[0] = 0.5 * (+xi + eta);
				coordsFineTriangle[1] = 0.5 * (-xi + eta);
			}
			else
			{
				Debug.Assert(subtriangle == 3);
				// |\
				// | \
				// | /
				// |/
				coordsFineTriangle[0] = 0.5 * (-xi + eta);
				coordsFineTriangle[1] = 0.5 * (-xi - eta);
			}

			// Calculate the shape functions in this fine element
			double[] shapeFunctions = FineElementInterpolation.EvaluateFunctionsAt(coordsFineTriangle);

			var result = new DualMeshPoint();
			result.FineNaturalCoordinates = coordsFineTriangle;
			result.FineShapeFunctions = shapeFunctions;

			result.FineElementIdx = new int[Dimension + 1];
			int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
			for (int d = 0; d < Dimension; ++d)
			{
				result.FineElementIdx[d] = coarseElementIdx[d];
			}
			result.FineElementIdx[Dimension] = subtriangle;

			return result;
		}

		//TODO: These mapping and its inverse must also work for points on edges/faces of the fine and coarse mesh.
		public override double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
		{
			// Map from the fine triangle to the coarse quad. 
			// To prove these formulas, use the interpolation from natural triangle to natural quad system.
			double r = coordsFineNatural[0];
			double s = coordsFineNatural[1];
			var coordsQuad = new double[2];
			if (fineElementIdx[2] == 0)
			{
				//   /\
				//  /  \
				// /____\
				coordsQuad[0] = -r + s;
				coordsQuad[1] = -r - s;
			}
			else if (fineElementIdx[2] == 1)
			{
				//   /|
				//  / |
				//  \ |
				//   \|
				coordsQuad[0] = +r + s;
				coordsQuad[1] = -r + s;
			}
			else if (fineElementIdx[2] == 2)
			{
				// ____
				// \  /
				//  \/
				coordsQuad[0] = +r - s;
				coordsQuad[1] = +r + s;
			}
			else
			{
				Debug.Assert(fineElementIdx[2] == 3);
				// |\
				// | \
				// | /
				// |/
				coordsQuad[0] = -r - s;
				coordsQuad[1] = +r - s;
			}
			return coordsQuad;
		}

		public class Builder
		{
			private readonly double[] minCoordinates;
			private readonly double[] maxCoordinates;
			private readonly int[] numNodesCoarse;
			private readonly int[] numNodesFine;
			private int axisMajorChoice = 0;

			public Builder(double[] minCoordinates, double[] maxCoordinates, int[] numNodesCoarse, int[] numNodesFine)
			{
				this.minCoordinates = minCoordinates;
				this.maxCoordinates = maxCoordinates;
				this.numNodesCoarse = numNodesCoarse;
				this.numNodesFine = numNodesFine;
			}

			/// <summary>
			/// The node IDs will be ordered such that they are contiguous along dimension <paramref name="axis"/>. Calling this
			/// method overrides the default node order: nodes are contiguous in the dimension with mininum number of nodes.
			/// </summary>
			/// <param name="axis">The axis along which node ids will be contiguous. 0 for x or 1 for y.</param>
			/// <returns>This object for chaining.</returns>
			public Builder SetMajorAxis(int axis)
			{
				this.axisMajorChoice = axis;
				return this;
			}

			public DualCartesianSimplicialSymmetricMesh2D BuildMesh()
			{
				int[] numElementsCoarse = { numNodesCoarse[0] - 1, numNodesCoarse[1] - 1 };
				var coarseMesh = new UniformCartesianMesh2D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
					.SetMajorAxis(axisMajorChoice) //TODO: Implement the other options in the mesh class and the builder.
					.BuildMesh();
				var fineMesh = new UniformSimplicialSymmetricMesh2D.Builder(minCoordinates, maxCoordinates, numNodesFine)
					.SetMajorAxis(axisMajorChoice)
					.BuildMesh();
				return new DualCartesianSimplicialSymmetricMesh2D(coarseMesh, fineMesh);
			}
		}
	}
}
