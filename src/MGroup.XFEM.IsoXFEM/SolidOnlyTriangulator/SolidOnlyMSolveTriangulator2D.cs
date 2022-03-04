namespace MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Geometry.Primitives;
	using MGroup.XFEM.Geometry.Tolerances;

	class SolidOnlyMSolveTriangulator2D : ISolidOnlyTriangulator
	{
		public Vector ElementNodalLevelSetValues { get; set; }
		public IXGeometryDescription LevelSetDescription { get ; set; }
		public Vector NodalLevelSetModel { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }

		IElementSubcell[] IConformingTriangulator.FindConformingMesh(
			IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
			=> FindConformingMesh(element, intersections, meshTolerance);

		public ElementSubtriangle2D[] FindConformingMesh(IXFiniteElement element,
			IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
		{
			// Store the nodes and all intersection points in a set
			double tol = meshTolerance.CalcTolerance(element);
			var comparer = new Point2DComparer(tol);
			var nodes = new SortedSet<double[]>(comparer);
			nodes.UnionWith(element.Interpolation.NodalNaturalCoordinates);

			// Store the nodes and all intersection points in a different set
			var triangleVertices = new SortedSet<double[]>(comparer);
			triangleVertices.UnionWith(nodes);

			// Add intersection points from each curve-element intersection object.
			foreach (IElementDiscontinuityInteraction intersection in intersections)
			{
				// If the curve does not intersect this element (e.g. it conforms to the element edge), 
				// there is no need to take into account for triangulation
				if (intersection.RelativePosition != RelativePositionCurveElement.Intersecting) continue;

				IList<double[]> newVertices = intersection.GetVerticesForTriangulation();
				int countBeforeInsertion = triangleVertices.Count;
				triangleVertices.UnionWith(newVertices);

				if (triangleVertices.Count == countBeforeInsertion)
				{
					// Corner case: the curve intersects the element at 2 opposite nodes. In this case also add the middle of their 
					// segment to force the Delauny algorithm to conform to the segment.
					//TODO: I should use constrained Delauny in all cases and conform to the intersection segment.
					double[] p0 = newVertices[0];
					double[] p1 = newVertices[1];
					if (nodes.Contains(p0) && nodes.Contains(p1))
					{
						triangleVertices.Add(new double[] { 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]) });
					}
				}
			}
			var triangulator = new MIConvexHullTriangulator2D();
			triangulator.MinTriangleArea = tol * element.CalcBulkSizeNatural();
			IList<Triangle2D> delaunyTriangles = triangulator.CreateMesh(triangleVertices);
			var subtriangles = new ElementSubtriangle2D[delaunyTriangles.Count];
			var solidSubTrianglesList = new List<ElementSubtriangle2D>();
			for (int t = 0; t < delaunyTriangles.Count; ++t)
			{
				subtriangles[t] = new ElementSubtriangle2D(delaunyTriangles[t]);
				double[] centroidNatural = subtriangles[t].FindCentroidNatural();
				XPoint centroid = new XPoint(centroidNatural.Length);
				centroid.Element = element;
				centroid.Coordinates[CoordinateSystem.ElementNatural] = centroidNatural;
				centroid.ShapeFunctions = element.Interpolation.EvaluateFunctionsAt(centroidNatural);
				double signedDistance = LevelSetDescription.SignedDistanceOf(centroid);
				if (signedDistance >= 0)
				{
					solidSubTrianglesList.Add(subtriangles[t]);
				}
			}
			ElementSubtriangle2D[] solidSubTriangles = new ElementSubtriangle2D[solidSubTrianglesList.Count];
			for (int i = 0; i < solidSubTrianglesList.Count; i++)
			{
				solidSubTriangles[i] = solidSubTrianglesList[i];
			}
			return solidSubTriangles;
		}
	}
}
