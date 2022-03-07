//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Text;
//using MGroup.MSolve.Meshes.Structured;
//using MGroup.XFEM.Interpolation;

//namespace MGroup.XFEM.Geometry.Mesh
//{
//	/// <summary>
//	/// This works only for a very specific division of 1 Hexa27 into 24 Tet4.
//	/// </summary>
//	public class DualCartesianSimplicialSymmetricMesh3D : DualCartesianSimplicialSymmetricMeshBase
//	{
//		private DualCartesianSimplicialSymmetricMesh3D(UniformCartesianMesh3D coarseMesh, UniformSimplicialSymmetricMesh3D fineMesh)
//			: base(3, coarseMesh, fineMesh, 24)
//		{
//		}

//		//HERE: check these
//		public override IIsoparametricInterpolation CoarseElementInterpolation { get; } = InterpolationHexa27.UniqueInstance;

//		public override IIsoparametricInterpolation FineElementInterpolation { get; } = InterpolationTet4.UniqueInstance;

//		public override DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords)
//		{
//			int subtetrahedron = FindSubcellIdx(coarseNaturalCoords);

//			// Map from the coarse quad to the fine triangle.
//			double xi = coarseNaturalCoords[0];
//			double eta = coarseNaturalCoords[1];
//			double zeta = coarseNaturalCoords[2];
//			double r, s, t;
//			if (subtetrahedron == 0)
//			{
//				r = 0.5 * (-xi - eta);
//				s = 0.5 * (+xi - eta);
//			}
//			else if (subtetrahedron == 1)
//			{
//				r = 0.5 * (+xi - eta);
//				s = 0.5 * (+xi + eta);
//			}
//			else if (subtetrahedron == 2)
//			{
//				r = 0.5 * (+xi + eta);
//				s = 0.5 * (-xi + eta);
//			}
//			else
//			{
//				Debug.Assert(subtetrahedron == 3);
//				r = 0.5 * (-xi + eta);
//				s = 0.5 * (-xi - eta);
//			}
//			double[] coordsFineTriangle = { r, s, t };

//			// Calculate the shape functions in this fine element
//			double[] shapeFunctions = FineElementInterpolation.EvaluateFunctionsAt(coordsFineTriangle);

//			var result = new DualMeshPoint();
//			result.FineNaturalCoordinates = coordsFineTriangle;
//			result.FineShapeFunctions = shapeFunctions;

//			result.FineElementIdx = new int[Dimension + 1];
//			int[] coarseElementIdx = coarseMesh.GetElementIdx(coarseElementID);
//			for (int d = 0; d < Dimension; ++d)
//			{
//				result.FineElementIdx[d] = coarseElementIdx[d];
//			}
//			result.FineElementIdx[Dimension] = subtriangle;

//			return result;
//		}

//		//TODO: These mapping and its inverse must also work for points on edges/faces of the fine and coarse mesh.
//		public override double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural)
//		{
//			// Map from the fine triangle to the coarse quad. 
//			// To prove these formulas, use the interpolation from natural triangle to natural quad system.
//			double r = coordsFineNatural[0];
//			double s = coordsFineNatural[1];
//			var coordsQuad = new double[2];
//			if (fineElementIdx[2] == 0)
//			{
//				//   /\
//				//  /  \
//				// /____\
//				coordsQuad[0] = -r + s;
//				coordsQuad[1] = -r - s;
//			}
//			else if (fineElementIdx[2] == 1)
//			{
//				//   /|
//				//  / |
//				//  \ |
//				//   \|
//				coordsQuad[0] = +r + s;
//				coordsQuad[1] = -r + s;
//			}
//			else if (fineElementIdx[2] == 2)
//			{
//				// ____
//				// \  /
//				//  \/
//				coordsQuad[0] = +r - s;
//				coordsQuad[1] = +r + s;
//			}
//			else
//			{
//				Debug.Assert(fineElementIdx[2] == 3);
//				// |\
//				// | \
//				// | /
//				// |/
//				coordsQuad[0] = -r - s;
//				coordsQuad[1] = +r - s;
//			}
//			return coordsQuad;
//		}

//		private static int FindSubcellIdx(double[] coarseNaturalCoords)
//		{
//			double xi = coarseNaturalCoords[0];
//			double eta = coarseNaturalCoords[1];
//			double zeta = coarseNaturalCoords[2];
//		}

//		public class Builder
//		{
//			private readonly double[] minCoordinates;
//			private readonly double[] maxCoordinates;
//			private readonly int[] numNodesCoarse;
//			private readonly int[] numNodesFine;
//			private int axisMajorChoice = 0;
//			private int axisMinorChoice = 2;

//			public Builder(double[] minCoordinates, double[] maxCoordinates, int[] numNodesCoarse, int[] numNodesFine)
//			{
//				this.minCoordinates = minCoordinates;
//				this.maxCoordinates = maxCoordinates;
//				this.numNodesCoarse = numNodesCoarse;
//				this.numNodesFine = numNodesFine;
//			}

//			/// <summary>
//			/// The node ids will be ordered such that they are contiguous along dimension <paramref name="majorAxis"/>, while they  
//			/// will have the maximum id difference along dimension <paramref name="minorAxis"/>. 
//			/// Calling this method overrides the default node order: nodes are contiguous in the dimension with mininum
//			/// number of nodes and have the maximum id difference in the dimension with the maximum number of nodes.
//			/// </summary>
//			/// <param name="majorAxis">
//			/// The axis along which node ids will be contiguous. 0 for x, 1 for y or 2 for z. 
//			/// Must be different from <paramref name="minorAxis"/>.
//			/// </param>
//			/// <param name="minorAxis">
//			/// The axis along which node ids will have the maximum distance (compared to other axes). 
//			/// 0 for x, 1 for y or 2 for z. Must be different from <paramref name="majorAxis"/>.
//			/// </param>
//			/// <returns>This object for chaining.</returns>
//			public Builder SetMajorMinorAxis(int majorAxis, int minorAxis)
//			{
//				this.axisMajorChoice = majorAxis;
//				this.axisMinorChoice = minorAxis;
//				return this;
//			}

//			public DualCartesianSimplicialSymmetricMesh3D BuildMesh()
//			{
//				int[] numElementsCoarse = { numNodesCoarse[0] - 1, numNodesCoarse[1] - 1, numNodesCoarse[2] - 1 };
//				var coarseMesh = new UniformCartesianMesh3D.Builder(minCoordinates, maxCoordinates, numElementsCoarse)
//					.SetMajorMinorAxis(axisMajorChoice, axisMinorChoice)
//					.BuildMesh();
//				var fineMesh = new UniformSimplicialSymmetricMesh3D.Builder(minCoordinates, maxCoordinates, numNodesFine)
//					.SetMajorMinorAxis(axisMajorChoice, axisMinorChoice)
//					.BuildMesh();
//				return new DualCartesianSimplicialSymmetricMesh3D(coarseMesh, fineMesh);
//			}
//		}
//	}
//}
