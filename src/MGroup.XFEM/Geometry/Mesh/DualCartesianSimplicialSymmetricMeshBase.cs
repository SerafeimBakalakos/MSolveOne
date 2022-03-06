using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.Geometry.Mesh
{
	public abstract class DualCartesianSimplicialSymmetricMeshBase : IDualMesh
	{
		protected readonly ICartesianMesh coarseMesh;
		protected readonly IStructuredMesh fineMesh;
		protected readonly int numFineElementsPerCoarseElement;
		protected readonly int numSimplicesPerCartesianCell;

		protected DualCartesianSimplicialSymmetricMeshBase(int dimension, ICartesianMesh coarseMesh, IStructuredMesh fineMesh,
			int numSimplicesPerCartesianCell) 
		{
			this.Dimension = dimension;
			this.coarseMesh = coarseMesh;
			this.fineMesh = fineMesh;
			this.numSimplicesPerCartesianCell = numSimplicesPerCartesianCell;

			for (int d = 0; d < Dimension; ++d)
			{
				if (fineMesh.NumNodes[d] - 1 != 2 * (coarseMesh.NumNodes[d] - 1))
				{
					throw new ArgumentException("Mismatch of nodes between the two meshes");
				}
			}

			numFineElementsPerCoarseElement = numSimplicesPerCartesianCell;
		}

		public int Dimension { get; }

		public IStructuredMesh FineMesh => fineMesh;

		public IStructuredMesh CoarseMesh => coarseMesh;

		public abstract IIsoparametricInterpolation CoarseElementInterpolation { get; }

		public abstract IIsoparametricInterpolation FineElementInterpolation { get; }

		public int[] FindSurroundingCoarseNodesForFineNode(int fineNodeID)
		{
			int[] fineNodeIdx = FineMesh.GetNodeIdx(fineNodeID);
			var coarseNodeIndicesPerAxis = new List<int>[Dimension];
			for (int d = 0; d < Dimension; d++)
			{
				coarseNodeIndicesPerAxis[d] = new List<int>(2);
				if (fineNodeIdx[d] % 2 == 0)
				{
					// Coarse and fine node idx coincide (in this axis)
					coarseNodeIndicesPerAxis[d].Add(fineNodeIdx[d] / 2); 
				}
				else
				{
					// Fine node idx lies between 2 fine node indices (in this axis)
					coarseNodeIndicesPerAxis[d].Add(fineNodeIdx[d] / 2);
					coarseNodeIndicesPerAxis[d].Add(fineNodeIdx[d] / 2 + 1);
				}
			}
			List<int[]> coarseNodeIndices = CombineIndices(coarseNodeIndicesPerAxis);

			var coarseNodeIDs = new int[coarseNodeIndices.Count];
			for (int n = 0; n < coarseNodeIndices.Count; ++n)
			{
				coarseNodeIDs[n] = CoarseMesh.GetNodeID(coarseNodeIndices[n]);
			}
			return coarseNodeIDs;
		}

		/// <summary>
		/// If the node in the fine mesh does not correspond to a node in the coarse mesh, -1 will be returned
		/// </summary>
		/// <param name="fineNodeID"></param>
		public int MapNodeFineToCoarse(int fineNodeID)
		{
			int[] fineIdx = FineMesh.GetNodeIdx(fineNodeID);

			for (int d = 0; d < Dimension; d++)
			{
				if (fineIdx[d] % 2 != 0)
				{
					return -1;
				}
			}

			var coarseIdx = new int[Dimension];
			for (int d = 0; d < Dimension; d++)
			{
				coarseIdx[d] = fineIdx[d] / 2;
			}
			return CoarseMesh.GetNodeID(coarseIdx);
		}

		public int MapNodeIDCoarseToFine(int coarseNodeID)
		{
			int[] coarseIdx = CoarseMesh.GetNodeIdx(coarseNodeID);
			var fineIdx = MapNodeIdxCoarseToFine(coarseIdx);
			return FineMesh.GetNodeID(fineIdx);
		}

		public int[] MapNodeIdxCoarseToFine(int[] coarseNodeIdx)
		{
			var fineIdx = new int[Dimension];
			for (int d = 0; d < Dimension; d++)
			{
				fineIdx[d] = 2 * coarseNodeIdx[d];
			}
			return fineIdx;
		}

		public int MapElementFineToCoarse(int fineElementID)
		{
			// The last entry is unused, since all sub-simplices belong to the same cartesian cell, defined by the first entries.
			int[] fineIdx = fineMesh.GetElementIdx(fineElementID);  
			var coarseIdx = new int[Dimension];
			for (int d = 0; d < Dimension; d++)
			{
				coarseIdx[d] = fineIdx[d];
			}
			return coarseMesh.GetElementID(coarseIdx);
		}

		public int[] MapElementCoarseToFine(int coarseElementID)
		{
			int[] coarseIdx = coarseMesh.GetElementIdx(coarseElementID);
			var fineElementIDs = new int[numFineElementsPerCoarseElement];
			for (int j = 0; j < numSimplicesPerCartesianCell; ++j)
			{
				var fineIdx = new int[Dimension + 1];
				for (int d = 0; d < Dimension; d++)
				{
					fineIdx[d] = coarseIdx[d];
				}
				fineIdx[Dimension] = j;
				int fineID = fineMesh.GetElementID(fineIdx);
				fineElementIDs[j] = fineID;
			}
			return fineElementIDs;
		}

		public abstract DualMeshPoint CalcShapeFunctions(int coarseElementID, double[] coarseNaturalCoords);

		//TODO: This mapping and its inverse must also work for points on edges of the fine and coarse mesh.
		public abstract double[] MapPointFineNaturalToCoarseNatural(int[] fineElementIdx, double[] coordsFineNatural);

		private static List<int[]> CombineIndices(List<int>[] indicesPerAxis)
		{
			var result = new List<int[]>();
			if (indicesPerAxis.Length == 2)
			{
				foreach (int j in indicesPerAxis[1])
				{
					foreach (int i in indicesPerAxis[0])
					{
						result.Add(new int[] { i, j });
					}
				}
			}
			else if (indicesPerAxis.Length == 3)
			{
				foreach (int k in indicesPerAxis[2])
				{
					foreach (int j in indicesPerAxis[1])
					{
						foreach (int i in indicesPerAxis[0])
						{
							result.Add(new int[] { i, j, k });
						}
					}
				}
			}
			else
			{
				throw new NotImplementedException();
			}
			return result;
		}
	}
}