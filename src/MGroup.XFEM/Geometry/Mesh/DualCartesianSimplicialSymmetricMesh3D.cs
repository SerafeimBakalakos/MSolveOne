using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.Mesh
{
	/// <summary>
	/// This works only for a very specific division of 1 Hexa27 into 24 Tet4.
	/// </summary>
	public class DualCartesianSimplicialSymmetricMesh3D : DualCartesianSimplicialSymmetricMeshBase
	{
		private DualCartesianSimplicialSymmetricMesh3D(UniformCartesianMesh3D coarseMesh, UniformSimplicialSymmetricMesh3D fineMesh)
			: base(3, coarseMesh, fineMesh, 24)
		{
		}

		//HERE: check these
		public override IIsoparametricInterpolation CoarseElementInterpolation { get; } = InterpolationHexa8.UniqueInstance;

		public override IIsoparametricInterpolation FineElementInterpolation { get; } = InterpolationTet4.UniqueInstance;

		public override DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
		{
			int subtetrahedron = FindSubcellIdx(coarseNaturalCoords);
			double[] coordsFineTetra = MapPointCoarseNaturalToFineNatural(subtetrahedron, coarseNaturalCoords);

			// Calculate the shape functions in this fine element
			double[] shapeFunctions = FineElementInterpolation.EvaluateFunctionsAt(coordsFineTetra);

			var result = new DualMeshPoint();
			result.FineNaturalCoordinates = coordsFineTetra;
			result.FineShapeFunctions = shapeFunctions;

			result.FineElementIdx = new int[Dimension + 1];
			int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
			for (int d = 0; d < Dimension; ++d)
			{
				result.FineElementIdx[d] = coarseElementIdx[d];
			}
			result.FineElementIdx[Dimension] = subtetrahedron;

			return result;
		}

		public int FindSubcellIdx(double[] coarseNaturalCoords)
		{
			double xi = coarseNaturalCoords[0];
			double eta = coarseNaturalCoords[1];
			double zeta = coarseNaturalCoords[2];

			if ((zeta <= eta) && (eta <= xi) && (eta <= -xi)) return 0;
			else if ((zeta <= -xi) && (eta >= -xi) && (eta <= xi)) return 1;
			else if ((zeta <= -eta) && (eta >= xi) && (eta >= -xi)) return 2;
			else if ((zeta <= xi) && (eta <= -xi) && (eta >= xi)) return 3;
			else if ((eta <= xi) && (zeta <= -xi) && (zeta >= xi)) return 4;
			else if ((zeta <= -eta) && (zeta >= -xi) && (zeta >= xi)) return 5;
			else if ((eta <= -xi) && (zeta >= -xi) && (zeta <= xi)) return 6;
			else if ((zeta >= eta) && (zeta <= -xi) && (zeta <= xi)) return 7;
			else if ((zeta >= xi) && (zeta <= eta) && (zeta <= -eta)) return 8;
			else if ((eta <= -xi) && (zeta >= -eta) && (zeta <= eta)) return 9;
			else if ((zeta <= -xi) && (zeta >= eta) && (zeta >= -eta)) return 10;
			else if ((eta >= xi) && (zeta <= -eta) && (zeta >= eta)) return 11;
			else if ((eta <= xi) && (zeta <= eta) && (zeta >= -eta)) return 12;
			else if ((zeta >= -xi) && (zeta <= -eta) && (zeta <= eta)) return 13;
			else if ((eta >= -xi) && (zeta >= eta) && (zeta <= -eta)) return 14;
			else if ((zeta <= xi) && (zeta >= -eta) && (zeta >= eta)) return 15;
			else if ((zeta <= eta) && (zeta >= xi) && (zeta >= -xi)) return 16;
			else if ((eta >= -xi) && (zeta <= -xi) && (zeta >= xi)) return 17;
			else if ((zeta >= -eta) && (zeta <= xi) && (zeta <= -xi)) return 18;
			else if ((eta >= xi) && (zeta >= -xi) && (zeta <= xi)) return 19;
			else if ((zeta >= xi) && (eta <= xi) && (eta >= -xi)) return 20;
			else if ((zeta >= -eta) && (eta <= -xi) && (eta <= xi)) return 21;
			else if ((zeta >= -xi) && (eta >= xi) && (eta <= -xi)) return 22;
			else if ((zeta >= eta) && (eta >= -xi) && (eta >= xi)) return 23;
			else throw new ArgumentException("This point does not belong to any subtetrahedron");
		}

		public double[] MapPointCoarseNaturalToFineNatural(int subtetrahedron, double[] coarseNaturalCoords)
		{
			double xi = coarseNaturalCoords[0];
			double eta = coarseNaturalCoords[1];
			double zeta = coarseNaturalCoords[2];

			if (subtetrahedron == 0) return new double[] { (-xi - eta) / 2, (xi - eta) / 2, zeta + 1 };
			else if (subtetrahedron == 1) return new double[] { (xi - eta) / 2, (xi + eta) / 2, zeta + 1 };
			else if (subtetrahedron == 2) return new double[] { (xi + eta) / 2, (-xi + eta) / 2, zeta + 1 };
			else if (subtetrahedron == 3) return new double[] { (-xi + eta) / 2, (-xi - eta) / 2, zeta + 1 };
			else if (subtetrahedron == 4) return new double[] { (-xi-zeta)/2, (-xi+zeta)/2, eta + 1 };
			else if (subtetrahedron == 5) return new double[] { (-xi + zeta) / 2, (xi+zeta)/2, eta + 1 };
			else if (subtetrahedron == 6) return new double[] { (xi + zeta) / 2, (xi - zeta) / 2, eta + 1 };
			else if (subtetrahedron == 7) return new double[] { (xi - zeta) / 2, (-xi - zeta) / 2, eta + 1 };
			else if (subtetrahedron == 8) return new double[] { (-eta-zeta)/2, (eta-zeta)/2, xi + 1};
			else if (subtetrahedron == 9) return new double[] { (eta - zeta) / 2, (eta + zeta) / 2, xi + 1 };
			else if (subtetrahedron == 10) return new double[] { (eta + zeta) / 2, (-eta + zeta) / 2, xi + 1 };
			else if (subtetrahedron == 11) return new double[] { (-eta + zeta) / 2, (-eta - zeta) / 2, xi + 1 };
			else if (subtetrahedron == 12) return new double[] { (eta + zeta) / 2, (eta - zeta) / 2, 1 - xi };
			else if (subtetrahedron == 13) return new double[] { (eta - zeta) / 2, (-eta - zeta) / 2, 1 - xi };
			else if (subtetrahedron == 14) return new double[] { (-eta - zeta) / 2, (-eta + zeta) / 2, 1 - xi };
			else if (subtetrahedron == 15) return new double[] { (-eta + zeta) / 2, (eta + zeta) / 2, 1 - xi };
			else if (subtetrahedron == 16) return new double[] { (xi + zeta) / 2, (-xi + zeta) / 2, 1 - eta };
			else if (subtetrahedron == 17) return new double[] { (-xi + zeta) / 2, (-xi - zeta) / 2, 1 - eta };
			else if (subtetrahedron == 18) return new double[] { (-xi - zeta) / 2, (xi - zeta) / 2, 1 - eta };
			else if (subtetrahedron == 19) return new double[] { (xi - zeta) / 2, (xi + zeta) / 2, 1 - eta };
			else if (subtetrahedron == 20) return new double[] { (xi + eta) / 2, (xi - eta) / 2, 1 - zeta };
			else if (subtetrahedron == 21) return new double[] { (xi - eta) / 2, (-xi - eta) / 2, 1 - zeta };
			else if (subtetrahedron == 22) return new double[] { (-xi - eta) / 2, (-xi + eta) / 2, 1 - zeta };
			else if (subtetrahedron == 23) return new double[] { (-xi + eta) / 2, (xi + eta) / 2, 1 - zeta };
			else throw new ArgumentException($"Invalid subtetrahedron. Must be [0,24), but was: {subtetrahedron}");
		}

		//TODO: These mapping and its inverse must also work for points on edges/faces of the fine and coarse mesh.
		public override double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
		{
			double r = coordsFineNatural[0];
			double s = coordsFineNatural[1];
			double t = coordsFineNatural[2];
			int subtetrahedron = fineElementIdx[3];

			if (subtetrahedron == 0) return new double[] { -r + s, -r - s, t - 1 };
			else if (subtetrahedron == 1) return new double[] { r + s, -r + s, t - 1 };
			else if (subtetrahedron == 2) return new double[] { r - s, r + s, t - 1 };
			else if (subtetrahedron == 3) return new double[] { -r - s, r - s, t - 1 };
			else if (subtetrahedron == 4) return new double[] { -r - s, t - 1, -r + s };
			else if (subtetrahedron == 5) return new double[] { -r + s, t - 1, r + s };
			else if (subtetrahedron == 6) return new double[] { r + s, t - 1, r - s };
			else if (subtetrahedron == 7) return new double[] { r - s, t - 1, -r - s };
			else if (subtetrahedron == 8) return new double[] { t - 1, -r + s, -r - s };
			else if (subtetrahedron == 9) return new double[] { t - 1, r + s, -r + s };
			else if (subtetrahedron == 10) return new double[] { t - 1, r - s, r + s };
			else if (subtetrahedron == 11) return new double[] { t - 1, -r - s, r - s };
			else if (subtetrahedron == 12) return new double[] { 1 - t, r + s, r - s };
			else if (subtetrahedron == 13) return new double[] { 1 - t, r - s, -r - s };
			else if (subtetrahedron == 14) return new double[] { 1 - t, -r - s, -r + s };
			else if (subtetrahedron == 15) return new double[] { 1 - t, -r + s, r + s };
			else if (subtetrahedron == 16) return new double[] { r - s, 1 - t, r + s };
			else if (subtetrahedron == 17) return new double[] { -r - s, 1 - t, r - s };
			else if (subtetrahedron == 18) return new double[] { -r + s, 1 - t, -r - s };
			else if (subtetrahedron == 19) return new double[] { r + s, 1 - t, -r + s };
			else if (subtetrahedron == 20) return new double[] { r + s, r - s, 1 - t };
			else if (subtetrahedron == 21) return new double[] { r - s, -r - s, 1 - t };
			else if (subtetrahedron == 22) return new double[] { -r - s, -r + s, 1 - t };
			else if (subtetrahedron == 23) return new double[] { -r + s, r + s, 1 - t };
			else throw new ArgumentException($"Invalid subtetrahedron. Must be [0,24), but was: {subtetrahedron}");
		}
		

		public class Builder
		{
			private readonly double[] minCoordinates;
			private readonly double[] maxCoordinates;
			private readonly int[] numNodesCoarse;
			private readonly int[] numNodesFine;
			private int axisMajorChoice = 0;
			private int axisMinorChoice = 2;

			public Builder(double[] minCoordinates, double[] maxCoordinates, int[] numNodesCoarse, int[] numNodesFine)
			{
				this.minCoordinates = minCoordinates;
				this.maxCoordinates = maxCoordinates;
				this.numNodesCoarse = numNodesCoarse;
				this.numNodesFine = numNodesFine;
			}

			/// <summary>
			/// The node ids will be ordered such that they are contiguous along dimension <paramref name="majorAxis"/>, while they  
			/// will have the maximum id difference along dimension <paramref name="minorAxis"/>. 
			/// Calling this method overrides the default node order: nodes are contiguous in the dimension with mininum
			/// number of nodes and have the maximum id difference in the dimension with the maximum number of nodes.
			/// </summary>
			/// <param name="majorAxis">
			/// The axis along which node ids will be contiguous. 0 for x, 1 for y or 2 for z. 
			/// Must be different from <paramref name="minorAxis"/>.
			/// </param>
			/// <param name="minorAxis">
			/// The axis along which node ids will have the maximum distance (compared to other axes). 
			/// 0 for x, 1 for y or 2 for z. Must be different from <paramref name="majorAxis"/>.
			/// </param>
			/// <returns>This object for chaining.</returns>
			public Builder SetMajorMinorAxis(int majorAxis, int minorAxis)
			{
				this.axisMajorChoice = majorAxis;
				this.axisMinorChoice = minorAxis;
				return this;
			}

			public DualCartesianSimplicialSymmetricMesh3D BuildMesh()
			{
				int[] numElementsCoarse = { numNodesCoarse[0] - 1, numNodesCoarse[1] - 1, numNodesCoarse[2] - 1 };
				var coarseMesh = new UniformCartesianMesh3D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
					.SetMajorMinorAxis(axisMajorChoice, axisMinorChoice)
					.BuildMesh();
				var fineMesh = new UniformSimplicialSymmetricMesh3D.Builder(minCoordinates, maxCoordinates, numNodesFine)
					.SetMajorMinorAxis(axisMajorChoice, axisMinorChoice)
					.BuildMesh();
				return new DualCartesianSimplicialSymmetricMesh3D(coarseMesh, fineMesh);
			}
		}
	}
}
