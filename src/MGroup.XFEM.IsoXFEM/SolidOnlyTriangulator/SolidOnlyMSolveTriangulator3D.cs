namespace MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator
{
	using System;
	using System.Collections.Generic;
	using System.Diagnostics;
	using System.Text;

	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Geometry.Primitives;
	using MGroup.XFEM.Geometry.Tolerances;

	public class SolidOnlyMSolveTriangulator3D : ISolidOnlyTriangulator
	{
		IElementSubcell[] IConformingTriangulator.FindConformingMesh(
		   IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
		   => FindConformingMesh(element, intersections, meshTolerance);

		public ElementSubtetrahedron3D[] FindConformingMesh(IXFiniteElement element,
			IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
		{
			//TODO: If an element is intersected by 2 surfaces which also intersect each other (inside the element) then this
			//      implementation is not sufficient to produce a conforming mesh.
			double pointProximityTolerance = meshTolerance.CalcTolerance(element);
			List<double[]> tetraVertices = FindTriangulationVertices(element, intersections, pointProximityTolerance);

			var triangulator = new MIConvexHullTriangulator3D();
			List<Tetrahedron3D> delaunyTetrahedra = triangulator.CreateMesh(tetraVertices);
			double minTetrahedronVolume = meshTolerance.CalcTolerance(element) * element.CalcBulkSizeNatural();
			//double minTetrahedronVolume = 1E-6;
			FilterTetrahedra(delaunyTetrahedra, minTetrahedronVolume);
			var subtetrahedra = new ElementSubtetrahedron3D[delaunyTetrahedra.Count];
			var solidSubtetrahedraList = new List<ElementSubtetrahedron3D>();
			for (int t = 0; t < delaunyTetrahedra.Count; ++t)
			{
				subtetrahedra[t] = new ElementSubtetrahedron3D(delaunyTetrahedra[t]);
				double[] centroidNatural = subtetrahedra[t].FindCentroidNatural();
				XPoint centroid = new XPoint(centroidNatural.Length);
				centroid.Element = element;
				centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
				centroid.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(centroidNatural);
				double signedDistance = LevelSetDescription.SignedDistanceOf(centroid);
				if (signedDistance >= 0)
				{
					solidSubtetrahedraList.Add(subtetrahedra[t]);
				}				
			}
			ElementSubtetrahedron3D[] solidSubtetrahedra3Ds= new ElementSubtetrahedron3D[solidSubtetrahedraList.Count];
			for (int i = 0; i < solidSubtetrahedraList.Count; i++)
			{
				solidSubtetrahedra3Ds[i] = solidSubtetrahedraList[i];
			}
			return solidSubtetrahedra3Ds;
		}

		private List<double[]> FindTriangulationVertices(IXFiniteElement element,
			IEnumerable<IElementDiscontinuityInteraction> intersections, double pointProximityTolerance)
		{
			// Store the nodes in a list
			var comparer = new Point3DComparer(pointProximityTolerance);
			var nodes = new List<double[]>(element.Interpolation.NodalNaturalCoordinates);

			// Store the nodes and all intersection points in a different list
			var tetraVertices = new List<double[]>(nodes);

			// Add intersection points from each curve-element intersection object.
			foreach (IElementDiscontinuityInteraction intersection in intersections)
			{
				// If the curve does not intersect this element (e.g. it conforms to the element edge), 
				// there is no need to take into account for triangulation
				if (intersection.RelativePosition != RelativePositionCurveElement.Intersecting) continue;

				IList<double[]> newVertices = intersection.GetVerticesForTriangulation();
				int countBeforeInsertion = tetraVertices.Count;
				Union(tetraVertices, newVertices, comparer);

				if (tetraVertices.Count == countBeforeInsertion)
				{
					// Corner case: the curve intersects the element at 4 opposite nodes. In this case also add their centroid 
					// to force the Delauny algorithm to conform to the segment. Or it intersects the element extremely close to
					// a node and MIConvexHull messes up with just the nodes of a Tet4 / Hexa8
					//TODO: I should use constrained Delauny in all cases and conform to the intersection segment.

					var centroid = new double[3];
					foreach (double[] node in tetraVertices)
					{
						for (int d = 0; d < 3; ++d) centroid[d] += node[d];
					}
					for (int d = 0; d < 3; ++d) centroid[d] /= tetraVertices.Count;
					tetraVertices.Add(centroid);

					//TODO: The next causes problems if these intersections are very close to each other. Is all this even necessary? 
					//      Can't we just take the centroid of the nodes or even take it directly as { 0, 0, 0 }?
					//bool areNodes = true;
					//var centroid = new double[3];
					//foreach (double[] vertex in newVertices)
					//{
					//    if (!Contains(nodes, vertex, comparer)) areNodes = false;
					//    for (int i = 0; i < 3; ++i)
					//    {
					//        centroid[i] += vertex[i];
					//    }
					//}
					//for (int i = 0; i < 3; ++i)
					//{
					//    centroid[i] /= newVertices.Count;
					//}

					//if (areNodes)
					//{
					//    tetraVertices.Add(centroid);
					//}
				}
			}

			return tetraVertices;
		}

		private void FilterTetrahedra(List<Tetrahedron3D> delaunyTetrahedra, double minTetrahedronVolume)
		{
			var tetrahedraToRemove = new List<Tetrahedron3D>();

			foreach (Tetrahedron3D tet in delaunyTetrahedra)
			{
				//TODO: Perhaps the cutoff criterion should take into account both the volume of the original element and the max volume of the subtets.
				// Remove very small tetrahedra
				double volume = Utilities.CalcTetrahedronVolume(tet.Vertices); 
				if (minTetrahedronVolume > 0)
				{
					if (Math.Abs(volume) < minTetrahedronVolume)
					{
						tetrahedraToRemove.Add(tet);
						continue;
					}
				}

				// MIConvexHull does not care about its tetrahedra having positive volume, so we need to enforce it.
				if (volume < 0)
				{
					// Swap the last 2 vertices, so that the volume changes sign.
					double[] swap = tet.Vertices[2];
					tet.Vertices[2] = tet.Vertices[3];
					tet.Vertices[3] = swap;
					Debug.Assert(Utilities.CalcTetrahedronVolume(tet.Vertices) > 0);
				}
			}
		}

		private bool Contains(IList<double[]> points, double[] newPoint, Point3DComparer comparer)
		{
			//TODO: This should be done by using a SortedSet and the comparer, but for some reason, it does not work correctly (since C#8)
			foreach (double[] oldPoint in points)
			{
				if (comparer.Compare(oldPoint, newPoint) == 0)
				{
					return true;
				}
			}
			return false;
		}

		private void Union(IList<double[]> existingPoints, IList<double[]> newPoints, Point3DComparer comparer)
		{
			//TODO: This should be done by using a SortedSet and the comparer, but for some reason, it keeps adding almost duplicate points
			foreach (double[] newPoint in newPoints)
			{
				bool pointExists = Contains(existingPoints, newPoint, comparer);
				if (!pointExists)
				{
					existingPoints.Add(newPoint);
				}
			}
		}	
	    public Vector ElementNodalLevelSetValues { get ; set; }
		public IXGeometryDescription LevelSetDescription { get ; set ; }

		private Dictionary<int, (XNode, bool)> SolidNodes(IXFiniteElement element)
	    {
			var solidNodes = new Dictionary<int, (XNode, bool)>();
			var m = 0;
			foreach (var node in element.Nodes)
			{
				if (ElementNodalLevelSetValues[m] < 0)
				{
					solidNodes[node.ID] = (node, false);
				}
				else
				{
					solidNodes[node.ID] = (node, true);
				}
			}
			return solidNodes;
	    }
		private List<(double, bool)> DistanceFromSolidNodes(Dictionary<int, (XNode, bool)> solidnodes, double[] centroid)
		{
			var distanceFromSolidNodes = new List<(double, bool)>();
			foreach (var nodeSolid in solidnodes.Values)
			{
				var distance = nodeSolid.Item1.CalculateDistanceFrom(centroid);
				var isDistanceFromSolidNodes = (distance, nodeSolid.Item2);
				distanceFromSolidNodes.Add(isDistanceFromSolidNodes);
			}
			return distanceFromSolidNodes;
		}
		
	}
}
