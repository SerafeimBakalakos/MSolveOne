using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Commons;

namespace MGroup.XFEM.Geometry
{
    public class IntersectionMesh2D_OLD : IIntersectionMesh
    {
        private const int dim = 2;

        public IntersectionMesh2D_OLD()
        {
        }

        public static IntersectionMesh2D_OLD CreateMesh(Dictionary<int, List<double[]>> intersectionsOfElements)
        {
            var mesh = new IntersectionMesh2D_OLD();
            if (intersectionsOfElements.Count == 0)
            {
                throw new ArgumentException("There must be at least 2 intersection points of at least 1 element");
            }
            else if (intersectionsOfElements.Count == 1)
            {
                if (intersectionsOfElements.First().Value.Count < 2)
                {
                    throw new ArgumentException("There must be at least 2 intersection points of at least 1 element");
                }
            }
            else
            {
                List<double[]> orderedPoints = OrderPoints(intersectionsOfElements);
                foreach (double[] point in orderedPoints) mesh.Vertices.Add(point);
                for (int i = 0; i < orderedPoints.Count - 1; ++i)
                {
                    mesh.Cells.Add((CellType.Line2, new int[] { i, i + 1 }));
                }
            }
            return mesh;
        }

        public int Dimension { get; } = dim;

        public IList<(CellType, int[])> Cells { get; } = new List<(CellType, int[])>();

        public IList<double[]> Vertices { get; } = new List<double[]>();

        public IIntersectionMesh MapToOtherSpace(Func<double[], double[]> mapVertex)
        {
            var result = new IntersectionMesh2D_OLD();
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
        public void MergeWith(IntersectionMesh2D_OLD other)
        {
            int offset = this.Vertices.Count;
            foreach (double[] vertex in other.Vertices) this.Vertices.Add(vertex);
            foreach ((CellType cellType, int[] originalConnectivity) in other.Cells)
            {
                int[] offsetConnectivity = OffsetArray(originalConnectivity, offset);
                this.Cells.Add((cellType, offsetConnectivity));
            }
        }

        private static List<double[]> OrderPoints(Dictionary<int, List<double[]>> intersectionsOfElements)
        {
            var remainingSegments = new List<List<double[]>>(intersectionsOfElements.Values);
            var orderedPoints = new LinkedList<double[]>();
            List<double[]> firstSegment = remainingSegments[0];
            remainingSegments.RemoveAt(0);
            orderedPoints.AddLast(firstSegment[0]);
            orderedPoints.AddLast(firstSegment[1]);

            double tol = 1E-6;
            var comparer = new ValueComparer(tol);
            while (remainingSegments.Count > 0)
            {
                var nextRemainingSegments = new List<List<double[]>>(remainingSegments.Count);
                foreach (List<double[]> segment in remainingSegments)
                {
                    double[] currentStart = orderedPoints.First.Value;
                    double[] currentEnd = orderedPoints.Last.Value;
                    if (PointsCoincide(segment[0], currentStart, comparer))
                    {
                        orderedPoints.AddFirst(segment[1]);
                    }
                    else if (PointsCoincide(segment[1], currentStart, comparer))
                    {
                        orderedPoints.AddFirst(segment[0]);
                    }
                    else if (PointsCoincide(segment[0], currentEnd, comparer))
                    {
                        orderedPoints.AddLast(segment[1]);
                    }
                    else if (PointsCoincide(segment[1], currentEnd, comparer))
                    {
                        orderedPoints.AddLast(segment[0]);
                    }
                    else
                    {
                        nextRemainingSegments.Add(segment);
                    }
                }

                if (nextRemainingSegments.Count == remainingSegments.Count)
                {
                    throw new Exception("The linear segments provided do not form a continuous mesh");
                }
                remainingSegments = nextRemainingSegments;
            }

            return orderedPoints.ToList();
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
