using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM
{
    /// <summary>
    /// Intersects a mesh of 2D elements in 3D with a level set surface . The level set surface must be orthogonal to the mesh
    /// and it's interpolation functions must be 1st order polynomials (linear).
    /// </summary>
    public class IntersectionMesh3DIntersector
    {
        private readonly IntersectionMesh3D_OLD oldMesh;
        private readonly double[] levelSetsOfOldVertices;

        //TODO: Perhaps depending on the order of "allVertices" and "allCells" can be avoided by using Dictionary
        private List<Vertex> allVertices; // Vertices from the old mesh are listed first in the same order as in the mesh.
        private List<Cell> allCells; // Cells from the old mesh are listed first in the same order as in the mesh.
        private List<Edge> oldEdges; 
        private IntersectionMesh3D_OLD newMesh;

        /// <summary>
        /// Warning if the <paramref name="levelSetsOfVertices"/> are calculated from a curved distance, then the intersection
        /// points of some edges will be at least slightly inaccurate. 
        /// </summary>
        /// <param name="originalMesh"></param>
        /// <param name="levelSetsOfVertices"></param>
        public IntersectionMesh3DIntersector(IntersectionMesh3D_OLD originalMesh, double[] levelSetsOfVertices)
        {
            this.oldMesh = originalMesh;
            this.levelSetsOfOldVertices = levelSetsOfVertices;
        }

        public IntersectionMesh3DIntersector(IntersectionMesh3D_OLD originalMesh, Plane3D intersectionPlane)
        {
            this.oldMesh = originalMesh;

            this.levelSetsOfOldVertices = new double[originalMesh.Vertices.Count];
            for (int v = 0; v < originalMesh.Vertices.Count; ++v)
            {
                levelSetsOfOldVertices[v] = intersectionPlane.SignedDistanceOf(originalMesh.Vertices[v]);
            }
        }

        public IntersectionMesh3D_OLD IntersectMesh()
        {
            // Create convenient Vertex, Cell, Edge objects to represent the old mesh
            ReencodeOldVerticesCells();
            FindOldEdgeConnectivities();

            SeparateVerticesKeptDiscarded();
            IntersectOldEdges();
            List<Cell> intersectedCells = SeparateCellsKeptIntersectedDiscarded();

            foreach (Cell cell in intersectedCells)
            {
                SplitCellIntoSubcells(cell);
            }

            return newMesh;
        }

        private static double DistanceBetween(Vertex vertex0, Vertex vertex1)
        {
            double sum = 0.0;
            for (int d = 0; d < 3; ++d)
            {
                double dx = vertex1.Coords[d] - vertex0.Coords[d];
                sum += dx * dx;
            }
            return Math.Sqrt(sum);
        }

        //TODO: Only do this for the edges of intersected elements
        private void FindOldEdgeConnectivities()
        {
            oldEdges = new List<Edge>();
            foreach (Cell cell in allCells)
            {
                if ((cell.CellType != CellType.Tri3) && (cell.CellType != CellType.Quad4)) throw new NotImplementedException();
                for (int v = 0; v < cell.Vertices.Count; ++v)
                {
                    Vertex point0 = cell.Vertices[v];
                    Vertex point1 = cell.Vertices[(v + 1) % cell.Vertices.Count];

                    // Check if this edge is already listed, otherwise create it.
                    Edge thisEdge = null;
                    foreach (Edge otherEdge in oldEdges)
                    {
                        if (otherEdge.Coincides(point0, point1))
                        {
                            thisEdge = otherEdge;
                            break;
                        }
                    }
                    if (thisEdge == null)
                    {
                        thisEdge = new Edge(point0, point1);
                        oldEdges.Add(thisEdge);
                    }

                    // Associate this edge with the cell
                    cell.Edges.Add(thisEdge);
                }
            }
        }
        
        private void GroupVerticesOfCellBasedOnPsi(Cell cell)
        {
            foreach (Vertex vertex in cell.Vertices)
            {
                if (levelSetsOfOldVertices[vertex.IdxOld] < 0)
                {
                    cell.VerticesWithNegativePsi.Add(vertex);
                }
                else if (levelSetsOfOldVertices[vertex.IdxOld] > 0)
                {
                    cell.VerticesWithPositivePsi.Add(vertex);
                }
                else
                {
                    cell.VerticesWithZeroPsi.Add(vertex);
                }
            }
        }

        /// <summary>
        /// Find intersection points of edges of the old mesh: 0 or 1 point on each edge.
        /// </summary>
        private void IntersectOldEdges()
        {
            foreach (Edge edge in oldEdges)
            {
                double psi0 = levelSetsOfOldVertices[edge.Start.IdxOld];
                double psi1 = levelSetsOfOldVertices[edge.End.IdxOld];

                // The following only applies if the signed distances are calculated from a curved surface:
                // Intersections that lie on edges of the original FE mesh are accurate. However intersections found on
                // other lines (edges of the triangles) are slightly wrong, unless the edges are parallel to xi or eta.
                // This is due to the fact that psi are calculated accurately from the curved initial geometry.
                // The next formula would be correct if psi0, psi1 were the distances from the linearized initial curve.
                if (psi0 * psi1 < 0)
                {
                    // There is 1 intersection point that does not coincide with nodes 
                    double[] x0 = edge.Start.Coords;
                    double[] x1 = edge.End.Coords;

                    var w = new double[3];
                    double ratio = -psi0 / (psi1 - psi0);
                    for (int d = 0; d < 3; ++d)
                    {
                        w[d] = x0[d] + ratio * (x1[d] - x0[d]);
                    }

                    // Create a new vertex
                    var intersection = new Vertex() { Coords = w, IdxNew = newMesh.Vertices.Count };
                    edge.Intersection = intersection;
                    newMesh.Vertices.Add(w);
                    //TODO: Perhaps inputing it to the new mesh should be done after I locate all intersection points and subtriangles
                }
            }
        }

        private void ReencodeOldVerticesCells()
        {
            allVertices = new List<Vertex>();
            for (int v = 0; v < oldMesh.Vertices.Count; ++v)
            {
                allVertices.Add(new Vertex() { Coords = oldMesh.Vertices[v], IdxOld = v });
            }

            allCells = new List<Cell>();
            for (int c = 0; c < oldMesh.Cells.Count; ++c)
            {
                CellType cellType = oldMesh.Cells[c].Item1;
                int[] vertexIndices = oldMesh.Cells[c].Item2;

                var cell = new Cell();
                cell.CellType = cellType;
                cell.IdxOld = c;
                foreach (int oldVertexIdx in vertexIndices)
                {
                    cell.Vertices.Add(allVertices[oldVertexIdx]);
                }
                allCells.Add(cell);
            }
        }

        //TODO: Perhaps this should be split into 2 methods: List<Vertex> CreateVertices(), void KeepOldVerticesWithNegativePsi();
        private void SeparateVerticesKeptDiscarded()
        {
            newMesh = new IntersectionMesh3D_OLD();
            for (int v = 0; v < oldMesh.Vertices.Count; ++v)
            {
                if (levelSetsOfOldVertices[v] <= 0)
                {
                    int newVertexIdx = newMesh.Vertices.Count;
                    newMesh.Vertices.Add(oldMesh.Vertices[v]);
                    allVertices[v].IdxNew = newVertexIdx;
                }
            }
        }

        private List<Cell> SeparateCellsKeptIntersectedDiscarded()
        {
            // Find which cells are intersected, which will be kept and which will be discarded
            var intersectedCells = new List<Cell>();
            foreach (Cell cell in allCells)
            {
                GroupVerticesOfCellBasedOnPsi(cell);

                if ((cell.VerticesWithNegativePsi.Count > 0) && (cell.VerticesWithPositivePsi.Count > 0))
                {
                    intersectedCells.Add(cell);

                    // Find intersection points of this cell
                    foreach (Edge edge in cell.Edges)
                    {
                        if (edge.Intersection != null)
                        {
                            cell.IntersectedEdges.Add(edge);
                            //cell.IntersectionPoints.Add(edge.Intersection);
                        }
                    }
                }
                else if (cell.VerticesWithPositivePsi.Count == 0)
                {
                    // Keep this cell as it is
                    var newCellVertices = new int[cell.Vertices.Count];
                    for (int v = 0; v < cell.Vertices.Count; ++v)
                    {
                        newCellVertices[v] = cell.Vertices[v].IdxNew;
                    }
                    newMesh.Cells.Add((cell.CellType, newCellVertices));
                }
            }
            return intersectedCells;
        }

        /// <summary>
        /// Split each triangle into 2 subcells and keep the correct subcell.
        /// The original cell is assumed to be intersected.
        /// </summary>
        /// <param name="cell"></param>
        private void SplitCellIntoSubcells(Cell cell) //TODO: split this into smaller methods
        {
            if (cell.CellType != CellType.Tri3) throw new NotImplementedException("The cells must be triangles");

            if (cell.VerticesWithZeroPsi.Count == 0) // The triangle is intersected at 2 points, which may lie on any 2 edges
            {
                Debug.Assert(cell.IntersectedEdges.Count == 2);

                // The triangle is divided into a subtriangle and a subquad
                if (cell.VerticesWithNegativePsi.Count == 1)
                {
                    // Case 0: The subtriangle lies in the region where psi<=0
                    // There is 1 vertex with psi<0 and 2 intersection points.
                    // Add this subtriangle to the new mesh.
                    int[] subtriangleVertices =
                    {
                        cell.VerticesWithNegativePsi[0].IdxNew,
                        cell.IntersectedEdges[0].Intersection.IdxNew,
                        cell.IntersectedEdges[1].Intersection.IdxNew
                    };
                    newMesh.Cells.Add((CellType.Tri3, subtriangleVertices));
                }
                else
                {
                    // Case 1: The subquad lies in the region where psi<=0.
                    // There are 2 vertices with psi<0 (Pi, Pj) and 2 intersection points (Pjk, Pik).
                    // Then (Pi, Pj, Pjk, Pik) defines the quad. First of all, find these points.
                    Debug.Assert(cell.VerticesWithNegativePsi.Count == 2);
                    Vertex pi = cell.VerticesWithNegativePsi[0];
                    Vertex pj = cell.VerticesWithNegativePsi[1];
                    Vertex pik, pjk;
                    if ((pi == cell.IntersectedEdges[0].Start) || (pi == cell.IntersectedEdges[0].End))
                    {
                        pik = cell.IntersectedEdges[0].Intersection;
                        pjk = cell.IntersectedEdges[1].Intersection;
                    }
                    else
                    {
                        Debug.Assert((pj == cell.IntersectedEdges[0].Start) || (pj == cell.IntersectedEdges[0].End));
                        pjk = cell.IntersectedEdges[0].Intersection;
                        pik = cell.IntersectedEdges[1].Intersection;
                    }

                    // Add two triangles to the mesh from these 4 points: avoid creating triangles with very sharp angles.
                    double diagonal0 = DistanceBetween(pi, pjk);
                    double diagonal1 = DistanceBetween(pj, pik);
                    if (diagonal0 < diagonal1)
                    {
                        newMesh.Cells.Add((CellType.Tri3, new int[] { pi.IdxNew, pj.IdxNew, pjk.IdxNew }));
                        newMesh.Cells.Add((CellType.Tri3, new int[] { pi.IdxNew, pjk.IdxNew, pik.IdxNew }));
                    }
                    else
                    {
                        newMesh.Cells.Add((CellType.Tri3, new int[] { pi.IdxNew, pj.IdxNew, pik.IdxNew }));
                        newMesh.Cells.Add((CellType.Tri3, new int[] { pj.IdxNew, pjk.IdxNew, pik.IdxNew }));
                    }

                }
            }
            else // Corner case: psi=0 goes through a vertex of this triangle (already found) and the opposite edge
            {
                Debug.Assert(cell.VerticesWithZeroPsi.Count == 1, "Otherwise it would not be an intersected element");
                Debug.Assert(cell.VerticesWithNegativePsi.Count == 1);
                Debug.Assert(cell.IntersectedEdges.Count == 1);

                // Add new triangle to the mesh: the 2 vertices with psi<=0 and the intersection point
                int[] newTriangleVertices =
                {
                    cell.VerticesWithZeroPsi[0].IdxNew,
                    cell.VerticesWithNegativePsi[0].IdxNew,
                    cell.IntersectedEdges[0].Intersection.IdxNew
                };
                newMesh.Cells.Add((CellType.Tri3, newTriangleVertices));
            }
        }

        private class Vertex
        {
            public double[] Coords { get; set; }

            public int IdxOld { get; set; } = -1;

            public int IdxNew { get; set; } = -1;

        }

        private class Cell
        {
            public CellType CellType { get; set; }

            public List<Edge> Edges { get; } = new List<Edge>();

            public int IdxOld { get; set; } = -1;

            public int IdxNew { get; set; } = -1;

            public List<Edge> IntersectedEdges { get; } = new List<Edge>();

            public List<Vertex> Vertices { get; } = new List<Vertex>();

            public List<Vertex> VerticesWithNegativePsi { get; } = new List<Vertex>();

            public List<Vertex> VerticesWithPositivePsi { get; } = new List<Vertex>();

            public List<Vertex> VerticesWithZeroPsi { get; } = new List<Vertex>();
        }

        private class Edge
        {
            public Edge(Vertex point0, Vertex point1)
            {
                if (point0.IdxOld < point1.IdxOld)
                {
                    Start = point0;
                    End = point1;
                }
                else
                {
                    Start = point1;
                    End = point0;
                }
            }

            public bool Coincides(Vertex point0, Vertex point1)
            {
                if ((point0.IdxOld == Start.IdxOld) && (point1.IdxOld == End.IdxOld)) return true;
                else if ((point1.IdxOld == Start.IdxOld) && (point0.IdxOld == End.IdxOld)) return true;
                else return false;
            }

            public Vertex Start { get; }

            public Vertex End { get; }

            public Vertex Intersection { get; set; }
        }
    }
}
