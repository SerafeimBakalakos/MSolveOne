using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Commons;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Exceptions;

//TODO: Specialization that only uses triangles, thus requiring less memory and checks
namespace MGroup.XFEM.Geometry
{
    public class IntersectionMesh3D_OLD : IIntersectionMesh
    {
        private const int dim = 3;

        public IntersectionMesh3D_OLD()
        {
        }

        public static IntersectionMesh3D_OLD CreateMultiCellMesh3D(Dictionary<double[], HashSet<ElementFace>> intersectionPoints)
        {
            var mesh = new IntersectionMesh3D_OLD();
            if (intersectionPoints.Count < 3) throw new ArgumentException("There must be at least 3 points");
            else if (intersectionPoints.Count == 3)
            {
                foreach (double[] point in intersectionPoints.Keys) mesh.Vertices.Add(point);
                mesh.Cells.Add((CellType.Tri3, new int[] { 0, 1, 2 }));
            }
            else
            {
                List<double[]> orderedPoints = OrderPoints3D(intersectionPoints);
                foreach (double[] point in orderedPoints) mesh.Vertices.Add(point);

                // Create triangles that contain the first points and 2 others
                for (int j = 1; j < orderedPoints.Count - 1; ++j)
                {
                    mesh.Cells.Add((CellType.Tri3, new int[] { 0, j, j + 1 }));
                }
            }
            return mesh;
        }

        public static IntersectionMesh3D_OLD CreateSingleCellMesh(CellType cellType, IReadOnlyList<double[]> intersectionPoints)
        {
            var mesh = new IntersectionMesh3D_OLD();
            for (int i = 0; i < intersectionPoints.Count; ++i)
            {
                mesh.Vertices.Add(intersectionPoints[i]);
            }
            int[] connectivity = Enumerable.Range(0, intersectionPoints.Count).ToArray();
            mesh.Cells.Add((cellType, connectivity));
            return mesh;
        }

        public static IntersectionMesh3D_OLD JoinMeshes(Dictionary<int, IntersectionMesh3D_OLD> intersectionsOfElements)
        {
            var jointMesh = new IntersectionMesh3D_OLD();
            foreach (IntersectionMesh3D_OLD mesh in intersectionsOfElements.Values)
            {
                int startVertices = jointMesh.Vertices.Count;
                var vertexIndicesOldToNew = new int[mesh.Vertices.Count];
                var vertexIsNew = new bool[mesh.Vertices.Count];

                // Add vertices of this partial mesh to the joint mesh 
                var comparer = new ValueComparer(1E-6);
                for (int i = 0; i < mesh.Vertices.Count; ++i)
                {
                    // Check all existing vertices of the joint mesh, in case they coincide
                    int newVertexPos = -1;
                    for (int j = 0; j < startVertices; ++j) // No need to check the vertices of this partial mesh
                    {
                        if (PointsCoincide(jointMesh.Vertices[j], mesh.Vertices[i], comparer))
                        {
                            newVertexPos = j;
                            break;
                        }
                    }

                    // If this vertex does not exist in the joint mesh, add it
                    if (newVertexPos == -1)
                    {
                        vertexIsNew[i] = true;
                        newVertexPos = jointMesh.Vertices.Count;
                        jointMesh.Vertices.Add(mesh.Vertices[i]);
                    }

                    // Note its new position
                    vertexIndicesOldToNew[i] = newVertexPos;
                }

                // Add cells of this partial mesh to the joint mesh
                foreach ((CellType cellType, int[] oldConnectivity) in mesh.Cells)
                {
                    //TODO: Check if there is already a cell of the same type that has the same vertices! 
                    //      This can happen if the LSM mesh conforms to the curve. 

                    // Each cell has an array containing the positions of its vertices in the list of mesh vertices. 
                    // This array must be updated to reflect the new positions in the list of joint mesh vertices.
                    var newConnectivity = new int[oldConnectivity.Length];
                    for (int i = 0; i < oldConnectivity.Length; ++i)
                    {
                        newConnectivity[i] = vertexIndicesOldToNew[oldConnectivity[i]];
                    }
                    jointMesh.Cells.Add((cellType, newConnectivity));
                }
            }
            return jointMesh;
        }

        public int Dimension { get; } = dim;

        public IList<(CellType, int[])> Cells { get; } = new List<(CellType, int[])>();

        public IList<double[]> Vertices { get; } = new List<double[]>();

        public IIntersectionMesh MapToOtherSpace(Func<double[], double[]> mapVertex)
        {
            var result = new IntersectionMesh3D_OLD();
            foreach (double[] vertex in this.Vertices)
            {
                result.Vertices.Add(mapVertex(vertex));
            }
            foreach ((CellType, int[]) cell in this.Cells)
            {
                result.Cells.Add(cell);
            }
            return result;
        }

        /// <summary>
        /// This method just gathers all cells and renumbers the vertices accordingly, 
        /// without taking intersecting cells into account. 
        /// </summary>
        /// <param name="other"></param>
        public void MergeWith(IntersectionMesh3D_OLD other)
        {
            int offset = this.Vertices.Count;
            foreach (double[] vertex in other.Vertices) this.Vertices.Add(vertex);
            foreach ((CellType cellType, int[] originalConnectivity) in other.Cells)
            {
                int[] offsetConnectivity = OffsetArray(originalConnectivity, offset);
                this.Cells.Add((cellType, offsetConnectivity));
            }
        }

        private static List<double[]> OrderPoints3D(Dictionary<double[], HashSet<ElementFace>> facesOfPoints)
        {
            var orderedPoints = new List<double[]>();
            List<double[]> leftoverPoints = facesOfPoints.Keys.ToList();

            // First point
            orderedPoints.Add(leftoverPoints[0]);
            leftoverPoints.RemoveAt(0);

            // Rest of the points
            while (leftoverPoints.Count > 0)
            {
                double[] pointI = orderedPoints[orderedPoints.Count - 1];
                HashSet<ElementFace> phasesI = facesOfPoints[pointI];
                int j = FindPointWithCommonFace(phasesI, leftoverPoints, facesOfPoints);
                if (j >= 0)
                {
                    orderedPoints.Add(leftoverPoints[j]);
                    leftoverPoints.RemoveAt(j);
                }
                else
                {
                    throw new InvalidElementGeometryIntersectionException(
                        "No other intersection point lies on the same face as the current point");
                }
            }

            // Make sure the last point and the first one lie on the same face
            var facesFirst = facesOfPoints[orderedPoints[0]];
            var facesLast = facesOfPoints[orderedPoints[orderedPoints.Count - 1]];
            if (!HaveCommonEntries(facesFirst, facesLast))
            {
                throw new InvalidElementGeometryIntersectionException("The first and last point do not lie on the same face");
            }

            return orderedPoints;
        }

        private static int FindPointWithCommonFace(HashSet<ElementFace> phasesI, List<double[]> leftoverPoints, 
            Dictionary<double[], HashSet<ElementFace>> facesOfPoints)
        {
            for (int j = 0; j < leftoverPoints.Count; ++j)
            {
                HashSet<ElementFace> phasesJ = facesOfPoints[leftoverPoints[j]];
                if (HaveCommonEntries(phasesI, phasesJ)) return j;
            }
            return -1;
        }

        private static bool HaveCommonEntries(HashSet<ElementFace> facesSet0, HashSet<ElementFace> facesSet1)
        {
            foreach (var entry in facesSet0)
            {
                if (facesSet1.Contains(entry)) return true;
            }
            return false;
        }

        private static int[] OffsetArray(int[] original, int offset)
        {
            var result = new int[original.Length];
            for (int i = 0; i < original.Length; i++)
            {
                result[i] = original[i] + offset;
            }
            return result;
        }

        private static bool PointsCoincide(double[] point0, double[] point1, ValueComparer comparer)
        {
            //TODO: Possibly add some tolerance
            for (int d = 0; d < dim; ++d)
            {
                if (!comparer.AreEqual(point0[d], point1[d]))
                {
                    return false;
                }
            }
            return true;
        }
    }
}
