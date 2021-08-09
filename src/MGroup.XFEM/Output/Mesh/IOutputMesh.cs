using System.Collections.Generic;
using MGroup.XFEM.Output.Vtk;

//TODO: Perhaps I should use IDs for vertices, cells and avoid any generics.
namespace MGroup.XFEM.Output.Mesh
{
    /// <summary>
    /// Stores vertices and cells used for output (so far visualization). Also defines associations between the vertices
    /// and cells of the output mesh and the corresponding ones of the original mesh used for the analysis.
    /// </summary>
    public interface IOutputMesh
    {
        int NumOutCells { get; }

        int NumOutVertices { get; }

        /// <summary>
        /// The order will be always the same.
        /// </summary>
        IEnumerable<VtkCell> OutCells { get; }

        /// <summary>
        /// The order will be always the same.
        /// </summary>
        SortedDictionary<int, VtkPoint> OutVertices { get; }

        ///// <summary>
        ///// The order will be always the same.
        ///// </summary>
        //IEnumerable<VtkCell> GetOutCellsForOriginal(ICell<XNode> originalCell);

        ///// <summary>
        ///// The order will be always the same.
        ///// </summary>
        //IEnumerable<VtkPoint> GetOutVerticesForOriginal(XNode originalVertex);
    }
}
