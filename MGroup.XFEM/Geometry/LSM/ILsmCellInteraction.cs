using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.LSM
{
    public interface ILsmCellInteraction
    {
        /// <summary>
        /// The intersection mesh, which may be empty.
        /// </summary>
        IntersectionMesh Mesh { get; }

        /// <summary>
        /// The relative position between the LSM manifold and the cell.
        /// </summary>
        RelativePositionCurveElement Position { get; }

        void Resolve();
    }

    public interface ILsmElementInteractionFactory
    {
        /// <summary>
        /// Finds the interaction between the LSM manifold and an element.
        /// </summary>
        /// <param name="nodeIDs">The global unique IDs of the nodes of the element.</param>
        /// <param name="nodeCoords">
        /// The coordinates of the element's nodes, in any coordinate system of interest. 
        /// The intersection points will be placed in that coordinate system too.</param>
        /// <param name="nodeLevelSets">The level set values of the element's nodes.</param>
        /// <param name="tolerance">
        /// The tolerance that controls how close an intersection point can get to the nodes of its edge.
        /// </param>
        /// <returns></returns>
        ILsmCellInteraction CreateNewInteraction(
            IList<int> nodeIDs, List<double[]> nodeCoords, List<double> nodeLevelSets, double tolerance);
    }
}
