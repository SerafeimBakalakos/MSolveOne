using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;
using MIConvexHull;

namespace MGroup.XFEM.Geometry.ConformingMesh
{
    public class MIConvexHullTriangulator3D : ITriangulator3D
    {
        private readonly double minDistanceTolerance;

        //TODO: Add precision controls as properties
        public MIConvexHullTriangulator3D(double minDistanceTolerance = 1E-10)
        {
            this.minDistanceTolerance = minDistanceTolerance;
        }

        public List<Tetrahedron3D> CreateMesh(IEnumerable<double[]> points)
        {
            // Gather the vertices
            List<double[]> vertices = points.ToList();

            // Call 3rd-party mesh generator
            IEnumerable<DefaultTriangulationCell<DefaultVertex>> meshCells = CallMIConvexHullDelauny(vertices);

            // Process the tetrahedral cells
            var tetrahedra = new List<Tetrahedron3D>();
            foreach (DefaultTriangulationCell<DefaultVertex> meshCell in meshCells)
            {
                DefaultVertex[] verticesOfTriangle = meshCell.Vertices;
                Debug.Assert(verticesOfTriangle.Length == 4);
                var tetra = new Tetrahedron3D();
                for (int v = 0; v < 4; ++v)
                {
                    tetra.Vertices[v] = verticesOfTriangle[v].Position;
                }
                tetrahedra.Add(tetra);
            }

            return tetrahedra;
        }

        private IEnumerable<DefaultTriangulationCell<DefaultVertex>> CallMIConvexHullDelauny(IList<double[]> vertices)
        {
            Debug.Assert((vertices != null) && vertices.Count >= 4, 
                "There must be at least 4 non-coincident vertices for 3D Dealuny to work.");
            try
            {
                return Triangulation.CreateDelaunay(vertices).Cells;
            }
            catch (NullReferenceException)
            {
                // This probably happened because 2 or more vertices are very close to each other. 
                //TODO: It is also possible that MIConvexHull failed at triangulating the Hexa8 cube in isoparametric without 
                //      any points inside the cube.

                List<double[]> uniqueVertices = FilterUniqueVertices(vertices);

                // Retry the triangulation
                return Triangulation.CreateDelaunay(uniqueVertices).Cells;
            }
        }

        private List<double[]> FilterUniqueVertices(IList<double[]> originalVertices)
        {
            // First find the min allowable distance of 2 distinct vertices.
            double maxDistance = double.MinValue;
            for (int i = 0; i < originalVertices.Count; ++i)
            {
                for (int j = i + 1; j < originalVertices.Count; ++j)
                {
                    double distance = Utilities.Distance3D(originalVertices[i], originalVertices[j]);
                    if (distance > maxDistance)
                    {
                        maxDistance = distance;
                    }
                }
            }
            double minDistance = minDistanceTolerance * maxDistance;

            // Remove coincident vertices. 
            //TODO: If one of the coincident vertices is on the face/edge/vertex of the isoparametric cube and the other is
            //      slightly off, keep the one that is.
            var uniqueVertices = new List<double[]>(originalVertices.Count);
            foreach (double[] originalVertex in originalVertices)
            {
                bool coincides = false;
                foreach (double[] uniqueVertex in uniqueVertices)
                {
                    double distance = Utilities.Distance3D(uniqueVertex, originalVertex);
                    if (distance < minDistance)
                    {
                        coincides = true;
                        break;
                    }
                }

                if (!coincides)
                {
                    uniqueVertices.Add(originalVertex);
                }
            }
            return uniqueVertices;
        }
    }
}
