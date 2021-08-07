using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Commons;

//TODO: avoid storing CellType per cell. It is the same for all cells.
namespace MGroup.XFEM.Geometry
{
    public class IntersectionMesh : IIntersectionMesh
    {
        public IntersectionMesh(int dimension)
        {
            if ((dimension != 2) && (dimension != 3))
            {
                throw new ArgumentException("Dimension must be 2 or 3");
            }
            this.Dimension = dimension;
        }

        public int Dimension { get; } //TODO: DEBUG only check of added vertices and cells based on this.

        public IList<(CellType type, int[] connectivity)> Cells { get; } = new List<(CellType, int[])>();

        public IList<double[]> Vertices { get; } = new List<double[]>();

        /// <summary>
        /// For each vertex v in <see cref="Vertices"/>: 
        /// If v lies on the edge between nodes with IDs i, j (where i &lt; j) then <see cref="IntersectedEdges"/>[v] = {i, j}. 
        /// If v coincides with node with ID = i, then <see cref="IntersectedEdges"/>[v] = {i}.
        /// If v lies on the interior of the intersected element, then <see cref="IntersectedEdges"/>[v] = empty.
        /// If not needed, it can be cleared to save memory.
        /// </summary>
        public IList<int[]> IntersectedEdges = new List<int[]>();

        public IIntersectionMesh MapToOtherSpace(Func<double[], double[]> mapVertex)
        {
            var result = new IntersectionMesh(Dimension);
            foreach (double[] vertex in this.Vertices)
            {
                result.Vertices.Add(mapVertex(vertex));
            }
            foreach ((CellType, int[]) cell in this.Cells)
            {
                result.Cells.Add(cell);
            }
            foreach (int[] edge in this.IntersectedEdges)
            {
                result.IntersectedEdges.Add(edge);
            }
            return result;
        }

        /// <summary>
        /// </summary>
        /// <param name="other"></param>
        /// <param name="isEdgeOrientationConsistent">
        /// Enables optimizations if the orientation of edges is always the same. E.g. first: node with min ID, 
        /// second: node with max ID.
        /// </param>
        public void MergeWith(IntersectionMesh other, bool isEdgeOrientationConsistent = false)
        {
            //TODO: If there are more than 2 coincident points, I should keep all of them in a separate dictionary and take the 
            //      average at the end
            //TODO: If there are identical cells, keep only 1 of them

            double tol = 1E-6;
            var comparer = new ValueComparer(tol);

            // Process vertices
            var otherVertexIndices = new int[other.Vertices.Count]; 
            for (int p = 0; p < other.Vertices.Count; ++p)
            {
                int[] otherEdge = other.IntersectedEdges[p];
                int existingVertexIdx = -1;
                if (otherEdge.Length == 0)
                {
                    // Internal point. Search existing vertices to see if it coincides with one of them.
                    for (int v = 0; v < this.Vertices.Count; ++v)
                    {
                        if (PointsCoincide(this.Vertices[v], other.Vertices[p], comparer))
                        {
                            existingVertexIdx = v;
                            break;
                        }
                    }
                }
                else
                {
                    Debug.Assert((otherEdge.Length == 1) || (otherEdge.Length == 2));

                    // Point on the edge between 2 nodes or exactly on a node. 
                    // Search existing vertices to see if it coincides with one of them.
                    for (int v = 0; v < this.Vertices.Count; ++v)
                    {
                        if (PointsCoincide(this.IntersectedEdges[v], other.IntersectedEdges[p], isEdgeOrientationConsistent))
                        {
                            existingVertexIdx = v;
                            break;
                        }
                    }
                }

                if (existingVertexIdx > -1) // keep existing vertex, but take the average
                {
                    otherVertexIndices[p] = existingVertexIdx;
                    AveragePoints(this.Vertices[existingVertexIdx], other.Vertices[p]);
                }
                else // add new vertex
                {
                    otherVertexIndices[p] = this.Vertices.Count;
                    this.Vertices.Add(other.Vertices[p]);
                    this.IntersectedEdges.Add(otherEdge);
                }
            }

            // Process cells
            foreach ((CellType type, int[] connectivity) in other.Cells)
            {
                // Associate cell with the new vertex indices
                for (int i = 0; i < connectivity.Length; ++i)
                {
                    connectivity[i] = otherVertexIndices[connectivity[i]];
                }
                this.Cells.Add((type, connectivity));
            }
        }

        /// <summary>
        /// </summary>
        /// <param name="point">The result will be stored here</param>
        /// <param name="otherPoint"></param>
        private void AveragePoints(double[] point, double[] otherPoint)
        {
            Debug.Assert(point.Length == Dimension);
            Debug.Assert(otherPoint.Length == Dimension);
            for (int d = 0; d < Dimension; ++d)
            {
                point[d] = 0.5 * (point[d] + otherPoint[d]);
            }
        }

        private bool PointsCoincide(double[] point0, double[] point1, ValueComparer comparer)
        {
            for (int d = 0; d < Dimension; ++d)
            {
                if (!comparer.AreEqual(point0[d], point1[d]))
                {
                    return false;
                }
            }
            return true;
        }

        private static bool PointsCoincide(int[] edgeOfPoint0, int[] edgeOfPoint1, bool isEdgeOrientationConsistent)
        {
            if (edgeOfPoint0.Length != edgeOfPoint1.Length) return false;
            if (edgeOfPoint0.Length == 1)
            {
                return edgeOfPoint0[0] == edgeOfPoint1[0];
            }
            else
            {
                Debug.Assert(edgeOfPoint0.Length == 2);
                bool areSame = (edgeOfPoint0[0] == edgeOfPoint1[0]) && (edgeOfPoint0[1] == edgeOfPoint1[1]);
                if (isEdgeOrientationConsistent)
                {
                    return areSame;
                }
                else // try both orientations
                {
                    return areSame || ((edgeOfPoint0[0] == edgeOfPoint1[1]) && (edgeOfPoint0[1] == edgeOfPoint1[0]));
                }
            }
        }
    }
}
