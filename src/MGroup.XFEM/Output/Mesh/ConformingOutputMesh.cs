using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Output.Vtk;

//TODO: In blending elements the FE interpolation is not accurate. The user should have the option to split them up into subcells.
//      This way the fields are calculated at more points/vertices of subcells and the plot is more accurate. 
//      Step and ridge enrichments do not need it.
//TODO: This is also the case for intersected elements. These are already divided into triangles/tetrahedra, but can be 
//      subdivided more. Step enrichment does not need it, but ridge and crack tips do.
namespace MGroup.XFEM.Output.Mesh
{
	public class ConformingOutputMesh : IOutputMesh
	{
		private readonly List<VtkCell> outCells;
		private readonly SortedDictionary<int, VtkPoint> outVertices;
		private readonly Dictionary<IXFiniteElement, HashSet<VtkCell>> original2OutCells;
		private readonly Dictionary<IXFiniteElement, HashSet<Subcell>> originalCells2Subcells;
		//private readonly Dictionary<XNode, HashSet<VtkPoint>> original2OutVertices;

		public ConformingOutputMesh(IXModel model)
		{
			List<XNode> originalVertices = model.Nodes.Values.ToList();
			List<IXFiniteElement> originalCells = model.EnumerateElements().ToList();

			this.outVertices = new SortedDictionary<int, VtkPoint>();
			this.outCells = new List<VtkCell>(originalCells.Count);
			this.original2OutCells = new Dictionary<IXFiniteElement, HashSet<VtkCell>>();
			this.originalCells2Subcells = new Dictionary<IXFiniteElement, HashSet<Subcell>>();

			int outVertexID = 0;
			for (int c = 0; c < originalCells.Count; ++c)
			{
				IXFiniteElement element = originalCells[c];
				CellType cellType = ((IElementType)element).CellType;
				original2OutCells[element] = new HashSet<VtkCell>();

				bool isIntersected = (element.ConformingSubcells != null) && (element.ConformingSubcells.Length > 0);
				if (isIntersected)
				{
					originalCells2Subcells[element] = new HashSet<Subcell>();
					foreach (IElementSubcell subcell in element.ConformingSubcells)
					{
						var subvertices = new List<VtkPoint>();
						foreach (double[] subvertex in subcell.FindVerticesCartesian(element))
						{
							var point = new VtkPoint(outVertexID++, subvertex);
							subvertices.Add(point);
							outVertices[point.ID] = point;
						}

						//TODO: The resulting triangle is Tri3 only for 1st order elements. Extend this.
						Debug.Assert(cellType == CellType.Tri3 || cellType == CellType.Quad4 
							|| cellType == CellType.Tet4 || cellType == CellType.Hexa8);
						var outCell = new VtkCell(subcell.CellType, subvertices);
						outCells.Add(outCell);
						original2OutCells[element].Add(outCell);
						originalCells2Subcells[element].Add(new Subcell(element, subcell, subvertices));
					}
				}
				else
				{
					var verticesOfCell = new VtkPoint[element.Nodes.Count];
					for (int i = 0; i < element.Nodes.Count; ++i)
					{
						XNode originalVertex = element.Nodes[i];
						var outVertex = new VtkPoint(outVertexID++, originalVertex.Coordinates);

						outVertices[outVertex.ID] = outVertex;
						//original2OutVertices[originalVertex].Add(outVertex);
						verticesOfCell[i] = outVertex;
					}
					var outCell = new VtkCell(cellType, verticesOfCell);
					outCells.Add(outCell);
					original2OutCells[element].Add(outCell);
				}
			}

			this.NumOutVertices = outVertices.Count;
			this.NumOutCells = outCells.Count;
		}

		public int NumOutCells { get; }

		public int NumOutVertices { get; }

		public IEnumerable<VtkCell> OutCells => outCells;

		public SortedDictionary<int, VtkPoint> OutVertices => outVertices;

		public IEnumerable<VtkCell> GetOutCellsForOriginal(IXFiniteElement originalCell)
			=> original2OutCells[originalCell];

		//public IEnumerable<VtkPoint> GetOutVerticesForOriginal(XNode originalVertex)
		//    => original2OutVertices[originalVertex];

		/// <summary>
		/// If the <paramref name="originalCell"/> is not intersected by any curve, then an empty collection will be returned.
		/// </summary>
		/// <param name="originalCell"></param>
		/// <returns></returns>
		public IEnumerable<Subcell> GetSubcellsForOriginal(IXFiniteElement originalCell)
		{
			bool isIntersected = originalCells2Subcells.TryGetValue(originalCell, out HashSet<Subcell> subtriangles);
			if (isIntersected) return subtriangles;
			else return new Subcell[0];
		}

		//TODO: This is could be derived from VtkCell. Right now there is both a Subtriangle and a VtkCell and they store the 
		//      same data.
		public class Subcell
		{
			public Subcell(IXFiniteElement parentElement, IElementSubcell originalSubcell, 
				IReadOnlyList<VtkPoint> outVertices)
			{
				this.OriginalSubcell = originalSubcell;
				this.OutVertices = outVertices;
				this.ParentElement = parentElement;
			}

			public IElementSubcell OriginalSubcell { get; }

			/// <summary>
			/// Same order as <see cref="ElementSubtriangle.VerticesNatural"/> of <see cref="OriginalSubcell"/>.
			/// </summary>
			public IReadOnlyList<VtkPoint> OutVertices { get; }

			public IXFiniteElement ParentElement { get; }
		}
	}
}
