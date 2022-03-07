using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;

namespace MGroup.MSolve.Meshes.Structured
{
	/// <summary>
	/// Structured mesh with uniform distances between points on a cartesian grid. The elements are Tet4 (simplices) generated
	/// by dividing each Hexa27 cell of an equivalent cartesian mesh into 24 subtetrahedra. The element indices are thus int[4], 
	/// where the first 3 entries are the index of the enclosing Hexa27 of an equivalent cartesian mesh and the third entry is 
	/// 0-23 to denote one of the 24 subtriangles of said Hexa27.
	/// </summary>
	public class UniformSimplicialSymmetricMesh3D : IStructuredMesh
	{
		private const int dim = 3;
		private const int numNodesPerElement = 4;
		private const int numSimplicesPerCartesianCell = 24;

		/// <summary>
		/// Coordinates of the nodes of a Hexa27 element in its natural system. They are integers on purpose.
		/// </summary>
		private static readonly int[][] hexa27NodeCoords = new int[27][]
		{
			new int[] { -1, -1, -1 }, // N0
			new int[] { +1, -1, -1 }, // N1
			new int[] { +1, +1, -1 }, // N2
			new int[] { -1, +1, -1 }, // N3
			new int[] { -1, -1, +1 }, // N4
			new int[] { +1, -1, +1 }, // N5
			new int[] { +1, +1, +1 }, // N6
			new int[] { -1, +1, +1 }, // N7

			new int[] {  0, -1, -1 }, // N8
			new int[] { -1,  0, -1 }, // N9
			new int[] { -1, -1,  0 }, // N10
			new int[] { +1,  0, -1 }, // N11
			new int[] { +1, -1,  0 }, // N12
			new int[] {  0, +1, -1 }, // N13
			new int[] { +1, +1,  0 }, // N14
			new int[] { -1, +1,  0 }, // N15
			new int[] {  0, -1, +1 }, // N16
			new int[] { -1,  0, +1 }, // N17
			new int[] { +1,  0, +1 }, // N18
			new int[] {  0, +1, +1 }, // N19

			new int[] {  0,  0, -1 }, // N20
			new int[] {  0, -1,  0 }, // N21
			new int[] { -1,  0,  0 }, // N22
			new int[] { +1,  0,  0 }, // N23
			new int[] {  0, +1,  0 }, // N24
			new int[] {  0,  0, +1 }, // N25

			new int[] {  0,  0,  0 }  // N26
		};

		private readonly int axisMajor;
		private readonly int axisMedium;
		private readonly int axisMinor;
		private readonly double[] dx;
		private readonly int firstElementID;
		private readonly int firstNodeID;
		private readonly int[] numCartesianCells;

		/// <summary>
		/// Given the index (i,j,k) of the first node of a Hexa27 with coordinates=(-1,-1,-1), this list contains, for each 
		/// subtetrahedron of the Hexa27, the index offsets relative to (i,j,k) of the nodes of the subtetrahedron. The offset of 
		/// node (i,j,k) is (0,0,0), the offset of (i+1, j, k+2) is (1,0,2) and so forth. Convention: the first node is the 
		/// centroid of the Hexa27, the second node is the centroid of a face of the Hexa27 and the last 2 nodes are 2 of the 
		/// corners.
		/// </summary>
		private readonly List<int[][]> elementNodeIdxOffsets; //TODO: Consider changing this to the natural coordinates of the nodes (but integers), namely 0, -1,+1. It will be easier to read.

		private UniformSimplicialSymmetricMesh3D(double[] minCoordinates, double[] maxCoordinates, int[] numNodes,
			int axisMajor, int axisMedium, int axisMinor, int firstNodeID, int firstElementID)
		{
			this.MinCoordinates = minCoordinates;
			this.MaxCoordinates = maxCoordinates;
			this.NumNodes = numNodes;
			NumNodesTotal = NumNodes[0] * NumNodes[1] * NumNodes[2];

			dx = new double[dim];
			for (int d = 0; d < dim; d++)
			{
				dx[d] = (maxCoordinates[d] - minCoordinates[d]) / (numNodes[d] - 1);
			}

			this.numCartesianCells = new int[dim];
			for (int d = 0; d < dim; ++d)
			{
				this.numCartesianCells[d] = (numNodes[d] - 1) / 2;
			}
			NumElementsTotal = numSimplicesPerCartesianCell * numCartesianCells[0] * numCartesianCells[1] * numCartesianCells[2];

			this.axisMajor = axisMajor;
			this.axisMedium = axisMedium;
			this.axisMinor = axisMinor;
			this.firstNodeID = firstNodeID;
			this.firstElementID = firstElementID;

			elementNodeIdxOffsets = new List<int[][]>(numSimplicesPerCartesianCell);
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 20, 0, 1, 26 })); // Tetrahedron  0 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 20, 1, 2, 26 })); // Tetrahedron  1 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 20, 2, 3, 26 })); // Tetrahedron  2 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 20, 3, 0, 26 })); // Tetrahedron  3

			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 21, 0, 4, 26 })); // Tetrahedron  4 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 21, 4, 5, 26 })); // Tetrahedron  5 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 21, 5, 1, 26 })); // Tetrahedron  6 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 21, 1, 0, 26 })); // Tetrahedron  7

			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 22, 0, 3, 26 })); // Tetrahedron  8 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 22, 3, 7, 26 })); // Tetrahedron  9 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 22, 7, 4, 26 })); // Tetrahedron 10 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 22, 4, 0, 26 })); // Tetrahedron 11

			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 23, 6, 2, 26 })); // Tetrahedron 12 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 23, 2, 1, 26 })); // Tetrahedron 13 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 23, 1, 5, 26 })); // Tetrahedron 14 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 23, 5, 6, 26 })); // Tetrahedron 15

			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 24, 6, 7, 26 })); // Tetrahedron 16 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 24, 7, 3, 26 })); // Tetrahedron 17 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 24, 3, 2, 26 })); // Tetrahedron 18 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 24, 2, 6, 26 })); // Tetrahedron 20

			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 25, 6, 5, 26 })); // Tetrahedron 21 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 25, 5, 4, 26 })); // Tetrahedron 22 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 25, 4, 7, 26 })); // Tetrahedron 23 
			elementNodeIdxOffsets.Add(GetNodeIdxOffsetsOfTriangle(new int[] { 25, 7, 6, 26 })); // Tetrahedron 24 
		}

		public CellType CellType => CellType.Tet4;

		public int Dimension => dim;

		public double[] MinCoordinates { get; }

		public double[] MaxCoordinates { get; }

		public int NumElementsTotal { get; }

		public int[] NumNodes { get; }

		public int NumNodesPerElement => numNodesPerElement;

		public int NumNodesTotal { get; }

		public IEnumerable<(int nodeID, double[] coordinates)> EnumerateNodes()
		{
			for (int k = 0; k < NumNodes[axisMinor]; ++k)
			{
				for (int j = 0; j < NumNodes[axisMedium]; ++j)
				{
					for (int i = 0; i < NumNodes[axisMajor]; ++i)
					{
						var idx = new int[dim];
						idx[axisMinor] = k;
						idx[axisMedium] = j;
						idx[axisMajor] = i;

						yield return (GetNodeID(idx), GetNodeCoordinates(idx));
					}
				}
			}
		}

		public IEnumerable<(int elementID, int[] nodeIDs)> EnumerateElements()
		{
			for (int elementID = firstElementID; elementID < firstElementID + NumElementsTotal; ++elementID)
			{
				yield return (elementID, GetElementConnectivity(elementID));
			}
		}

		public int GetNodeID(int[] nodeIdx)
		{
			CheckNodeIdx(nodeIdx);

			// E.g. x-major, y-medium, z-minor: id = iX + iY * numNodesX + iZ * NumNodesX * NumNodesY
			return firstNodeID + nodeIdx[axisMajor] + nodeIdx[axisMedium] * NumNodes[axisMajor]
				+ nodeIdx[axisMinor] * NumNodes[axisMajor] * NumNodes[axisMedium];
		}

		public int[] GetNodeIdx(int nodeID)
		{
			CheckNodeID(nodeID);

			int id = nodeID - firstNodeID;
			int numNodesPlane = NumNodes[axisMajor] * NumNodes[axisMedium];
			int mod = id % numNodesPlane;

			var idx = new int[dim];
			idx[axisMinor] = id / numNodesPlane;
			idx[axisMedium] = mod / NumNodes[axisMajor];
			idx[axisMajor] = mod % NumNodes[axisMajor];

			return idx;
		}

		public double[] GetNodeCoordinates(int[] nodeIdx)
		{
			CheckNodeIdx(nodeIdx);
			var coords = new double[dim];
			for (int d = 0; d < dim; d++)
			{
				coords[d] = MinCoordinates[d] + nodeIdx[d] * dx[d];
			}
			return coords;
		}

		public int GetElementID(int[] elementIdx)
		{
			CheckElementIdx(elementIdx);
			int cartesianCellID = elementIdx[axisMajor] + elementIdx[axisMedium] * numCartesianCells[axisMajor]
				+ elementIdx[axisMinor] * numCartesianCells[axisMajor] * numCartesianCells[axisMedium];
			return firstElementID + numSimplicesPerCartesianCell * cartesianCellID + elementIdx[dim];
		}

		public int[] GetElementIdx(int elementID)
		{
			CheckElementID(elementID);

			var elementIdx = new int[dim + 1];
			elementIdx[dim] = (elementID - firstElementID) % numSimplicesPerCartesianCell;
			int cartesianCellID = (elementID - firstElementID) / numSimplicesPerCartesianCell;

			int numElementsPlane = numCartesianCells[axisMajor] * numCartesianCells[axisMedium];
			int mod = cartesianCellID % numElementsPlane;

			elementIdx[axisMinor] = cartesianCellID / numElementsPlane;
			elementIdx[axisMedium] = mod / numCartesianCells[axisMajor];
			elementIdx[axisMajor] = mod % numCartesianCells[axisMajor];
			return elementIdx;
		}

		public int[] GetElementConnectivity(int[] elementIdx)
		{
			CheckElementIdx(elementIdx);
			var nodeIDs = new int[numNodesPerElement];
			var nodeIdx = new int[dim]; // Avoid allocating an array per node
			for (int n = 0; n < numNodesPerElement; ++n)
			{
				int[][] offsetsOfTriangleNodes = elementNodeIdxOffsets[elementIdx[dim]];
				int[] offsetOfNode = offsetsOfTriangleNodes[n];
				nodeIdx[0] = 2 * elementIdx[0] + offsetOfNode[0];
				nodeIdx[1] = 2 * elementIdx[1] + offsetOfNode[1];
				nodeIdx[2] = 2 * elementIdx[2] + offsetOfNode[2];
				nodeIDs[n] = GetNodeID(nodeIdx);
			}

			return nodeIDs;
		}

		public int[] GetElementConnectivity(int elementID) => GetElementConnectivity(GetElementIdx(elementID));

		/// <summary>
		/// </summary>
		/// <param name="nodesOfTriangle">Three of the nodes 0-26 of the surrounding Hexa27.</param>
		/// <returns></returns>
		private static int[][] GetNodeIdxOffsetsOfTriangle(int[] nodesOfTriangle) //TODO: This method can be generalized for 2D and 3D
		{
			// E.g. Node0 (-1, -1, -1) of a Hexa27 is (i, j, k). Node1 (+1, -1, -1) is (i+1, j+0, k+0). 
			// Therefore we add +1 to the natural coordinates to get the offset from Node0.
			var result = new int[numNodesPerElement][];
			for (int n = 0; n < numNodesPerElement; ++n)
			{
				int[] nodeCoords = hexa27NodeCoords[nodesOfTriangle[n]];
				result[n] = new int[] { nodeCoords[0] + 1, nodeCoords[1] + 1, nodeCoords[2] + 1 };
			}
			return result;
		}

		[Conditional("DEBUG")]
		private void CheckElementIdx(int[] elementIdx)
		{
			if (elementIdx.Length != dim + 1)
			{
				throw new ArgumentException($"Element index must be an array with length = {dim + 1}");
			}
			for (int d = 0; d < dim; ++d)
			{
				if ((elementIdx[d] < 0) || (elementIdx[d] >= numCartesianCells[d]))
				{
					throw new ArgumentException($"Element index entry {d} must belong in [0, {numCartesianCells[d]})");
				}
			}
			if ((elementIdx[dim] < 0) || (elementIdx[dim] >= numSimplicesPerCartesianCell))
			{
				throw new ArgumentException($"Element index entry {dim} must belong in [0, {numSimplicesPerCartesianCell})");
			}
		}

		[Conditional("DEBUG")]
		private void CheckElementID(int elementID)
		{
			if ((elementID < firstElementID) || (elementID >= firstElementID + NumElementsTotal))
			{
				throw new ArgumentException(
					$"Element ID must belong in [{firstElementID}, {firstElementID + NumElementsTotal})");
			}
		}

		[Conditional("DEBUG")]
		private void CheckNodeIdx(int[] nodeIdx)
		{
			if (nodeIdx.Length != dim)
			{
				throw new ArgumentException($"Node index must be an array with length = {dim}");
			}
			for (int d = 0; d < dim; ++d)
			{
				if ((nodeIdx[d] < 0) || (nodeIdx[d] >= NumNodes[d]))
				{
					throw new ArgumentException($"Node index along dimension {d} must belong in [0, {NumNodes[d]})");
				}
			}
		}

		[Conditional("DEBUG")]
		private void CheckNodeID(int nodeID)
		{
			if ((nodeID < firstNodeID) || (nodeID >= firstNodeID + NumNodesTotal))
			{
				throw new ArgumentException(
					$"Node ID must belong in [{firstNodeID}, {firstNodeID + NumNodesTotal})");
			}
		}

		public class Builder
		{

			private readonly double[] coordsMin;
			private readonly double[] coordsMax;
			private readonly int[] numNodes;
			private int axisMajorChoice;
			private int axisMinorChoice;
			private int firstElementID;
			private int firstNodeID;
			private bool isAxisMajorMinorDefault;

			/// <summary>
			/// 
			/// </summary>
			/// <param name="minCoordinates"></param>
			/// <param name="maxCoordinates"></param>
			/// <param name="numNodes">Array with 3 positive integers.</param>
			public Builder(double[] minCoordinates, double[] maxCoordinates, int[] numNodes)
			{
				this.coordsMin = minCoordinates.Copy();
				this.coordsMax = maxCoordinates.Copy();
				this.numNodes = numNodes.Copy();

				// Defaults
				isAxisMajorMinorDefault = true;
				axisMajorChoice = int.MinValue;
				axisMinorChoice = int.MinValue;
				firstNodeID = 0;
				firstElementID = 0;
			}

			public UniformSimplicialSymmetricMesh3D BuildMesh()
			{
				Validate();
				(int majorAxis, int mediumAxis, int minorAxis) = ChooseAxes();
				ValidateAxes(majorAxis, mediumAxis, minorAxis);

				return new UniformSimplicialSymmetricMesh3D(coordsMin.Copy(), coordsMax.Copy(), numNodes.Copy(),
					majorAxis, mediumAxis, minorAxis, firstNodeID, firstElementID);
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
				this.isAxisMajorMinorDefault = false;
				this.axisMajorChoice = majorAxis;
				this.axisMinorChoice = minorAxis;
				return this;
			}

			/// <summary>
			/// Reapplies the default order of node ids: nodes are contiguous in the dimension with mininum number of nodes
			/// and have the maximum id difference in the dimension with the maximum number of nodes.
			/// </summary>
			/// <returns>This object for chaining.</returns>
			public Builder SetMajorMinorAxisDefault()
			{
				this.isAxisMajorMinorDefault = true;
				this.axisMajorChoice = int.MinValue;
				this.axisMinorChoice = int.MinValue;
				return this;
			}

			private (int majorAxis, int mediumAxis, int minorAxis) ChooseAxes()
			{
				int majorAxis, mediumAxis, minorAxis;
				if (isAxisMajorMinorDefault)
				{
					// Decide based on the number of nodes/elements per axis 
					// Sort axes based on their number of elements
					var entries = new List<(int count, int axis)>();
					entries.Add((numNodes[0], 0));
					entries.Add((numNodes[1], 1));
					entries.Add((numNodes[2], 2));
					int[] sortedAxes = SortAxes(entries);

					majorAxis = sortedAxes[0];
					mediumAxis = sortedAxes[1];
					minorAxis = sortedAxes[2];
				}
				else
				{
					// Respect client decision
					majorAxis = axisMajorChoice;
					minorAxis = axisMinorChoice;
					if ((majorAxis == 0) && (minorAxis == 1)) mediumAxis = 2;
					else if ((majorAxis == 0) && (minorAxis == 2)) mediumAxis = 1;
					else if ((majorAxis == 1) && (minorAxis == 0)) mediumAxis = 2;
					else if ((majorAxis == 1) && (minorAxis == 2)) mediumAxis = 0;
					else if ((majorAxis == 2) && (minorAxis == 0)) mediumAxis = 1;
					else if ((majorAxis == 2) && (minorAxis == 1)) mediumAxis = 0;
					else throw new ArgumentException("Major and minors axes must be 0, 1 or 2 and different from each other");
				}
				return (majorAxis, mediumAxis, minorAxis);
			}

			private static int[] SortAxes(List<(int count, int axis)> entries)
			{
				Debug.Assert(entries.Count == 3);
				var sortedAxes = new int[3];
				int idx = 0;
				while (idx < 3)
				{
					int min = int.MaxValue;
					int axisOfMin = -1;

					foreach ((int count, int axis) in entries)
					{
						if (count < min)
						{
							min = count;
							axisOfMin = axis;
						}
						else if (count == min) // prefer axis x over y, z and axis y over z
						{
							if (axis < axisOfMin)
							{
								min = count;
								axisOfMin = axis;
							}
						}
					}

					sortedAxes[idx++] = axisOfMin;
					bool removedEntry = entries.Remove((min, axisOfMin));
					Debug.Assert(removedEntry);
				}
				return sortedAxes;
			}

			//TODO: This must be called lazily but only once when building several products with the same builder without
			//      mutating the builder.
			private void Validate()
			{
				// Coordinates
				if (coordsMin.Length != dim) throw new ArgumentException($"Length of min coordinates must be {dim}.");
				if (coordsMax.Length != dim) throw new ArgumentException($"Length of max coordinates must be {dim}.");
				for (int d = 0; d < dim; ++d)
				{
					if (coordsMin[d] >= coordsMax[d])
					{
						throw new ArgumentException(
							$"Along axis {d}, min coordinates must be strictly less than max coordinates.");
					}
				}

				// Nodes per axis
				if (numNodes.Length != dim) throw new ArgumentException($"Length of number of nodes must be {dim}.");
				for (int d = 0; d < dim; ++d)
				{
					if (numNodes[d] < 3)
					{
						throw new ArgumentException($"Along axis {d}, there must be at least 3 nodes.");
					}
					if (numNodes[d] % 2 == 0)
					{
						throw new ArgumentException($"Along axis {d}, there must be an odd number nodes.");
					}
				}

				// Major, medium, minor axes will be checked in a different method
				// First node & element IDs: no illegal values yet.
			}

			private static void ValidateAxes(int majorAxis, int mediumAxis, int minorAxis)
			{
				bool isCorrect = false;
				isCorrect |= (majorAxis == 0) && (minorAxis == 1) && (mediumAxis == 2);
				isCorrect |= (majorAxis == 0) && (minorAxis == 2) && (mediumAxis == 1);
				isCorrect |= (majorAxis == 1) && (minorAxis == 0) && (mediumAxis == 2);
				isCorrect |= (majorAxis == 1) && (minorAxis == 2) && (mediumAxis == 0);
				isCorrect |= (majorAxis == 2) && (minorAxis == 0) && (mediumAxis == 1);
				isCorrect |= (majorAxis == 2) && (minorAxis == 1) && (mediumAxis == 0);

				if (!isCorrect)
				{
					throw new ArgumentException("Major and minors axes must be 0, 1 or 2 and different from each other");
				}
			}
		}
	}
}
