namespace MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator
{
	using System;
	using System.Collections;
	using System.Collections.Generic;
	using System.Linq;
	using System.Text;

	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.ElementGeometry;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Geometry.Primitives;
	using MGroup.XFEM.Geometry.Tolerances;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;

	public class SolidOnlyTriangulator3D : ISolidOnlyTriangulator
	{
		public Vector NodalLevelSetModel { get; set; }
		public Vector ElementNodalLevelSetValues { get ; set ; }
		public IXGeometryDescription LevelSetDescription { get ; set ; }

		public IsoXfemElementSubtetrahedon3D[] CreateSubTetrahedra(IXFiniteElement element)
		{
			var subtetrahedra = new IsoXfemElementSubtetrahedon3D[24];
			//IReadOnlyList<double[]> nodesNatural = element.Interpolation.NodalNaturalCoordinates;
			List<double[]> centreOfFaces = new List<double[]>();
			var centroid = XFiniteElementExtensions.FindCentroidNatural(element);
			int numOfSubTet4 = 0;
			foreach (var face in element.Faces)
			{
				var coordinatesOfFaces = face.NodesNatural;
				var centreofFace = Utilities.FindCentroid(coordinatesOfFaces);
				centreOfFaces.Add(centreofFace);
				foreach (var edge in face.Edges)
				{
					subtetrahedra[numOfSubTet4] = new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(edge.NodesNatural[0], edge.NodesNatural[1], centreofFace, centroid));
					subtetrahedra[numOfSubTet4].NodalLevelSetValues = CalcNodalLevelSetOfSubTet4(edge, centreofFace, centroid, element);
					subtetrahedra[numOfSubTet4].DefinePhaseOfElement();
					subtetrahedra[numOfSubTet4].ID = numOfSubTet4;
					numOfSubTet4++;
				}
			}
			return subtetrahedra;
		//	List<double[]> coordinatesAll = new List<double[]>();
		//	coordinatesAll.AddRange(nodesNatural);
		//	coordinatesAll.AddRange(centreOfFaces);
		//	coordinatesAll.Add(centroid);			
		//	subtetrahedra[0] = new ElementSubtetrahedron3D(new Tetrahedron3D(coordinatesAll[0], coordinatesAll[1], coordinatesAll[9], coordinatesAll[14]));
		}
		
		public (List<double[]> ,List<int>,List<int[]>)  FindIntersection(IsoXfemElementSubtetrahedon3D boundaryTet4, IXFiniteElement element)
		{
			List<double[]> coordinatesOfIntersectionPoints = new List<double[]>();
			//List<int> indexOfPositiveNodes = new List<int>();
			int numberOfIntersectionPointsOfBrick = 0;
			List<int[]> indexOfIntersectionNodes = new List<int[]>();
			List<int> nodesWithPositiveValues = new List<int>();
			int numberIntersectionPointsOfTet4 = 0;
			int[,] edgesOfTet4 = new int[,] { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
			for (int i = 0; i < boundaryTet4.NodalLevelSetValues.Length; i++)
			{
				if (boundaryTet4.NodalLevelSetValues[i]>0)
				{
					nodesWithPositiveValues.Add(i);
				}
			}
			for (int i = 0; i < edgesOfTet4.GetLength(0); i++)
			{
				if (boundaryTet4.NodalLevelSetValues[edgesOfTet4[i, 0]] * boundaryTet4.NodalLevelSetValues[edgesOfTet4[i, 1]] < 0)
				{
					numberIntersectionPointsOfTet4++;
					numberOfIntersectionPointsOfBrick++;
						if (boundaryTet4.NodalLevelSetValues[edgesOfTet4[i, 0]] > 0)
						{
							var indxOfIntersection = new int[] { edgesOfTet4[i, 0], edgesOfTet4[i, 1] };
							indexOfIntersectionNodes.Add(indxOfIntersection);
						}
						else
						{
							var indxOfIntersection = new int[] { edgesOfTet4[i, 1], edgesOfTet4[i, 0] };
							indexOfIntersectionNodes.Add(indxOfIntersection);
						}
					     var x1 = new double[3];
				          x1[0] = boundaryTet4.VerticesNatural[indexOfIntersectionNodes[numberIntersectionPointsOfTet4-1][0]][0];
					      x1[1] = boundaryTet4.VerticesNatural[indexOfIntersectionNodes[numberIntersectionPointsOfTet4 - 1][0]][1];
					      x1[2] = boundaryTet4.VerticesNatural[indexOfIntersectionNodes[numberIntersectionPointsOfTet4 - 1][0]][2];
					      var x2 = new double[3];
					      x2[0] = boundaryTet4.VerticesNatural[indexOfIntersectionNodes[numberIntersectionPointsOfTet4-1][1]][0];
					      x2[1] = boundaryTet4.VerticesNatural[indexOfIntersectionNodes[numberIntersectionPointsOfTet4 - 1][1]][1];
					      x2[2] = boundaryTet4.VerticesNatural[indexOfIntersectionNodes[numberIntersectionPointsOfTet4 - 1][1]][2];
					     var rel = Math.Abs(boundaryTet4.NodalLevelSetValues[indexOfIntersectionNodes[numberIntersectionPointsOfTet4-1][0]] / boundaryTet4.NodalLevelSetValues[indexOfIntersectionNodes[numberIntersectionPointsOfTet4-1][1]]);
						 var intersectionPointCoordinates = new double[3];
						 intersectionPointCoordinates[0] = x1[0] + (x2[0] - x1[0]) * rel / (1 + rel);
						 intersectionPointCoordinates[1] = x1[1] + (x2[1] - x1[1]) * rel / (1 + rel);
						 intersectionPointCoordinates[2] = x1[2] + (x2[2] - x1[2]) * rel / (1 + rel);
					    var shapeFunctionValuesIntersection = EvaluateAt(intersectionPointCoordinates);
						var relativeCriteriaIntersection = 0.0;
						for (int k = 0; k < ElementNodalLevelSetValues.Length; k++)
						{
							relativeCriteriaIntersection += ElementNodalLevelSetValues[k] * shapeFunctionValuesIntersection[k];
						}
						var relativeCriteriaFirstNode = ElementNodalLevelSetValues[indexOfIntersectionNodes[numberIntersectionPointsOfTet4-1][0]];
						var relativeCriteriaSecondNode = ElementNodalLevelSetValues[indexOfIntersectionNodes[numberIntersectionPointsOfTet4-1][1]];
					//Stabilize intersection coordinates
					while (Math.Abs(relativeCriteriaIntersection / new double[] { Math.Abs(relativeCriteriaFirstNode), Math.Abs(relativeCriteriaSecondNode) }.Average()) > 0.0001)
					{
						if (relativeCriteriaIntersection < 0)
						{
							x2[0] = intersectionPointCoordinates[0];
							x2[1] = intersectionPointCoordinates[1];
							x2[2] = intersectionPointCoordinates[2];
							relativeCriteriaSecondNode = relativeCriteriaIntersection;
						}
						else
						{
							x1[0] = intersectionPointCoordinates[0];
							x1[1] = intersectionPointCoordinates[1];
							x1[2] = intersectionPointCoordinates[2];
							relativeCriteriaFirstNode = relativeCriteriaIntersection;
						}
						rel = Math.Abs(relativeCriteriaFirstNode / relativeCriteriaSecondNode);
						intersectionPointCoordinates[0] = x1[0] + (x2[0] - x1[0]) * rel / (1 + rel);
						intersectionPointCoordinates[1] = x1[1] + (x2[1] - x1[1]) * rel / (1 + rel);
						intersectionPointCoordinates[2] = x1[2] + (x2[2] - x1[2]) * rel / (1 + rel);
						var newshapeFunctionValuesIntersection = EvaluateAt(intersectionPointCoordinates);
						relativeCriteriaIntersection = 0.0;
						for (int k = 0; k < ElementNodalLevelSetValues.Length; k++)
						{
							relativeCriteriaIntersection += ElementNodalLevelSetValues[k] * newshapeFunctionValuesIntersection[k];
						}
					}
					coordinatesOfIntersectionPoints.Add(intersectionPointCoordinates);
				}
			}
			return (coordinatesOfIntersectionPoints, nodesWithPositiveValues, indexOfIntersectionNodes);
		}
		public double[] EvaluateAt(double[] naturalPoint)
		{
			double xi = naturalPoint[0];
			double eta = naturalPoint[1];
			double zeta = naturalPoint[2];
			var oneOverEight = 0.125;
			var values = new double[8];
			values[0] = oneOverEight * (1 - xi) * (1 - eta) * (1 - zeta);
			values[1] = oneOverEight * (1 + xi) * (1 - eta) * (1 - zeta);
			values[2] = oneOverEight * (1 + xi) * (1 + eta) * (1 - zeta);
			values[3] = oneOverEight * (1 - xi) * (1 + eta) * (1 - zeta);
			values[4] = oneOverEight * (1 - xi) * (1 - eta) * (1 + zeta);
			values[5] = oneOverEight * (1 + xi) * (1 - eta) * (1 + zeta);
			values[6] = oneOverEight * (1 + xi) * (1 + eta) * (1 + zeta);
			values[7] = oneOverEight * (1 - xi) * (1 + eta) * (1 + zeta);
			return values;
		}
		public Vector CalcNodalLevelSetOfSubTet4(ElementEdge edge, double[] centreOfFace, double[] centroid, IXFiniteElement element)
		{
			Vector nodalLevelSetSubTet4 = Vector.CreateZero(4);
			nodalLevelSetSubTet4[0] = NodalLevelSetModel[edge.NodeIDs[0]];
			nodalLevelSetSubTet4[1] = NodalLevelSetModel[edge.NodeIDs[1]];
			var shapeFunctionValuesCentreOfFace = EvaluateAt(centreOfFace);
			var shapeFunctionValuesCentroid = EvaluateAt(centroid);
			for (int i = 0; i < ElementNodalLevelSetValues.Length; i++)
			{
				nodalLevelSetSubTet4[2] = nodalLevelSetSubTet4[2] + ElementNodalLevelSetValues[i] * shapeFunctionValuesCentreOfFace[i];
				nodalLevelSetSubTet4[3] = nodalLevelSetSubTet4[3] + ElementNodalLevelSetValues[i] * shapeFunctionValuesCentroid[i];
			}
			return nodalLevelSetSubTet4;
		}
		
		public (List<IsoXfemElementSubtetrahedon3D> solidSubtetrahedra3D, List<IsoXfemElementSubtetrahedon3D> boundarySubtetrahedra3D) ClassifySubtetrahedra (IsoXfemElementSubtetrahedon3D[] subtetrahedra3D)
		{
			List<IsoXfemElementSubtetrahedon3D> solidSubtetrahedra3D = new List<IsoXfemElementSubtetrahedon3D>();
			List<IsoXfemElementSubtetrahedon3D> boundarySubtetrahedra3D = new List<IsoXfemElementSubtetrahedon3D>();
			for (int i = 0; i < subtetrahedra3D.Length; i++)
			{
				if (subtetrahedra3D[i].PhaseOfSubTet4 == IsoXfemElementSubtetrahedon3D.Phase.solidSubTet4)
				{
					solidSubtetrahedra3D.Add(subtetrahedra3D[i]);
				}
				else if (subtetrahedra3D[i].PhaseOfSubTet4 == IsoXfemElementSubtetrahedon3D.Phase.boundarySubTet4)
				{
					boundarySubtetrahedra3D.Add(subtetrahedra3D[i]);
				}
			}
			return (solidSubtetrahedra3D, boundarySubtetrahedra3D);
		}

		//public IElementSubcell[] FindConformingMesh(IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance) => throw new NotImplementedException();
		public IElementSubcell[] FindConformingMesh(IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
		{
			var subTets4 = CreateSubTetrahedra(element);
			var (solidsubTets4, boundarysubTets4) = ClassifySubtetrahedra(subTets4);
			foreach (var boundarysubTet in boundarysubTets4)
			{
				var (intersectionPoints, positiveNodes, indexOfIntersectionNodes) = FindIntersection(boundarysubTet, element);
				if (intersectionPoints.Count == 3 & positiveNodes.Count == 1)
				{
					var solidpart = new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(intersectionPoints[0], intersectionPoints[1], intersectionPoints[2], boundarysubTet.VerticesNatural[positiveNodes[0]]));
					solidsubTets4.Add(solidpart);
				}
				else if (intersectionPoints.Count == 3 & positiveNodes.Count == 3)
				{
					var triangles = new Triangle2D[]
					{ new Triangle2D(boundarysubTet.VerticesNatural[positiveNodes[0]], boundarysubTet.VerticesNatural[positiveNodes[1]], boundarysubTet.VerticesNatural[positiveNodes[2]]),
					  new Triangle2D(intersectionPoints[0], intersectionPoints[1], intersectionPoints[2])};
					var coordinatesOfFirstQuad = new List<double[]>();
					coordinatesOfFirstQuad.Add(intersectionPoints[0]);
					coordinatesOfFirstQuad.Add(intersectionPoints[1]);
					coordinatesOfFirstQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[1][0]]);
					coordinatesOfFirstQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[0][0]]);
					var coordinatesOfSecQuad = new List<double[]>();
					coordinatesOfSecQuad.Add(intersectionPoints[1]);
					coordinatesOfSecQuad.Add(intersectionPoints[2]);
					coordinatesOfSecQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[2][0]]);
					coordinatesOfSecQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[1][0]]);
					var coordinatesOfThirdQuad = new List<double[]>();
					coordinatesOfThirdQuad.Add(intersectionPoints[2]);
					coordinatesOfThirdQuad.Add(intersectionPoints[0]);
					coordinatesOfThirdQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[0][0]]);
					coordinatesOfThirdQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[2][0]]);
					var quadrilaterals = new ConvexPolygon2D[]
					{ new ConvexPolygon2D(coordinatesOfFirstQuad),
					  new ConvexPolygon2D(coordinatesOfSecQuad),
					  new ConvexPolygon2D(coordinatesOfThirdQuad)};
					var allTriangles = new List<Triangle2D>();
					allTriangles.AddRange(triangles);
					//for (int i = 0; i < quadrilaterals.Length; i++)
					//{
					//	allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[0], quadrilaterals[i].Vertices[1], quadrilaterals[i].Vertices[2]));
					//}
					//for (int i = 0; i < quadrilaterals.Length; i++)
					//{
					//	allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[2], quadrilaterals[i].Vertices[3], quadrilaterals[i].Vertices[0]));
					//}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[0], quadrilaterals[i].Vertices[1], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[1], quadrilaterals[i].Vertices[2], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[2], quadrilaterals[i].Vertices[3], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[3], quadrilaterals[i].Vertices[0], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					var coordinates = new List<double[]>();
					coordinates.AddRange(new double[][] { boundarysubTet.VerticesNatural[positiveNodes[0]], boundarysubTet.VerticesNatural[positiveNodes[1]], boundarysubTet.VerticesNatural[positiveNodes[2]] });
					coordinates.AddRange(new double[][] { intersectionPoints[0], intersectionPoints[1], intersectionPoints[2] });
					var centre = Utilities.FindCentroid((IEnumerable<double[]>)coordinates);
					foreach (var triangle in allTriangles)
					{
						solidsubTets4.Add(new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(triangle.Vertices[0], triangle.Vertices[1], triangle.Vertices[2], centre)));
					}
				}
				else if (intersectionPoints.Count == 4)
				{
					var a1 = new List<int>();
					for (int i = 0; i < indexOfIntersectionNodes.Count; i++)
					{
						if (indexOfIntersectionNodes[i][0] == indexOfIntersectionNodes[0][0])
						{
							a1.Add(i);
						}
					}
					var arrayA1 = a1.ToArray();
					var b1 = ArraysMethods.SetDiff(new int[] { 0 }, arrayA1);
					var a2 = new List<int>();
					for (int i = 0; i < indexOfIntersectionNodes.Count; i++)
					{
						if (indexOfIntersectionNodes[i][1] == indexOfIntersectionNodes[b1[0]][1])
						{
							a2.Add(i);
						}
					}
					var arrayA2 = a2.ToArray();
					var b2 = ArraysMethods.SetDiff(new int[] { b1[0] }, arrayA2);
					var b3 = ArraysMethods.SetDiff(new int[] { 0, b1[0], b2[0] }, new int[] { 0, 1, 2, 3 });
					var triangles = new Triangle2D[]
					{ new Triangle2D(intersectionPoints[0], intersectionPoints[b1[0]], boundarysubTet.VerticesNatural[indexOfIntersectionNodes[0][0]]),
					  new Triangle2D(intersectionPoints[b2[0]], intersectionPoints[b3[0]], boundarysubTet.VerticesNatural[indexOfIntersectionNodes[b2[0]][0]])};
					var coordinatesOfFirstQuad = new List<double[]>();
					coordinatesOfFirstQuad.Add(intersectionPoints[0]);
					coordinatesOfFirstQuad.Add(intersectionPoints[b1[0]]);
					coordinatesOfFirstQuad.Add(intersectionPoints[b2[0]]);
					coordinatesOfFirstQuad.Add(intersectionPoints[b3[0]]);
					var coordinatesOfSecQuad = new List<double[]>();
					coordinatesOfSecQuad.Add(intersectionPoints[b1[0]]);
					coordinatesOfSecQuad.Add(intersectionPoints[b2[0]]);
					coordinatesOfSecQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[b3[0]][0]]);
					coordinatesOfSecQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[0][0]]);
					var coordinatesOfThirdQuad = new List<double[]>();
					coordinatesOfThirdQuad.Add(intersectionPoints[b3[0]]);
					coordinatesOfThirdQuad.Add(intersectionPoints[0]);
					coordinatesOfThirdQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[0][0]]);
					coordinatesOfThirdQuad.Add(boundarysubTet.VerticesNatural[indexOfIntersectionNodes[b3[0]][0]]);
					var quadrilaterals = new ConvexPolygon2D[]
					{ new ConvexPolygon2D(coordinatesOfFirstQuad),
					  new ConvexPolygon2D(coordinatesOfSecQuad),
					  new ConvexPolygon2D(coordinatesOfThirdQuad)};
					var allTriangles = new List<Triangle2D>();
					allTriangles.AddRange(triangles);
					//for (int i = 0; i < quadrilaterals.Length; i++)
					//{
					//	allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[0], quadrilaterals[i].Vertices[1], quadrilaterals[i].Vertices[2]));
					//}
					//for (int i = 0; i < quadrilaterals.Length; i++)
					//{
					//	allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[2], quadrilaterals[i].Vertices[3], quadrilaterals[i].Vertices[0]));
					//}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[0], quadrilaterals[i].Vertices[1], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[1], quadrilaterals[i].Vertices[2], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[2], quadrilaterals[i].Vertices[3], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					for (int i = 0; i < quadrilaterals.Length; i++)
					{
						allTriangles.Add(new Triangle2D(quadrilaterals[i].Vertices[3], quadrilaterals[i].Vertices[0], Utilities.FindCentroid((IEnumerable<double[]>)quadrilaterals[i].Vertices)));
					}
					var coordinates = new List<double[]>();
					for (int i = 0; i < positiveNodes.Count; i++)
					{
						coordinates.Add(boundarysubTet.VerticesNatural[positiveNodes[i]]);

					}
					coordinates.AddRange(new double[][] { intersectionPoints[0], intersectionPoints[1], intersectionPoints[2], intersectionPoints[3] });
					var centre = Utilities.FindCentroid((IEnumerable<double[]>)coordinates);
					foreach (var triangle in allTriangles)
					{
						solidsubTets4.Add(new IsoXfemElementSubtetrahedon3D(new Tetrahedron3D(triangle.Vertices[0], triangle.Vertices[1], triangle.Vertices[2], centre)));
					}
				}				
			}
			ElementSubtetrahedron3D[] solidTets4 = new ElementSubtetrahedron3D[solidsubTets4.Count] ;
			for (int i = 0; i < solidsubTets4.Count; i++)
			{
				solidTets4[i] = new ElementSubtetrahedron3D(new Tetrahedron3D(solidsubTets4[i].VerticesNatural[0], solidsubTets4[i].VerticesNatural[1], solidsubTets4[i].VerticesNatural[2], solidsubTets4[i].VerticesNatural[3]));
			}
			return solidTets4;
		}
	}
}
