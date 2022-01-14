namespace MGroup.XFEM.IsoXFEM
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Geometry.Primitives;
	using MGroup.XFEM.Geometry.Tolerances;

	public class SolidOnlyTriangulator2D : ISolidOnlyTriangulator
	{
		public Vector ElementNodalLevelSetValues { get ; set ; }

		public Tuple<Vector, Matrix, Vector> IntersectionPoints(Matrix elementCoordinates, Vector elementNodalLevelSet)
		{
			Vector nodalLevelSetOnFirstNode = elementNodalLevelSet.GetSubvector(new int[] { 0 });
			int[] elementNodesCircular = new int[] { 0, 1, 2, 3, 0 };
			Vector nodalLevelSetCircular = elementNodalLevelSet.Append(nodalLevelSetOnFirstNode);
			int numOfIntersectionPoints = 0; // no of intersection points
											 //Nodes which have a boundary intersection in between
			List<int[]> allPointsWithIntersection = new List<int[]>();
			for (int i = 0; i < 4; i++)
			{
				if (nodalLevelSetCircular[i] * nodalLevelSetCircular[i + 1] < 0)
				{
					int[] intersectionPoints = { elementNodesCircular[i], elementNodesCircular[i + 1] };
					allPointsWithIntersection.Add(intersectionPoints);
					numOfIntersectionPoints++;
				}
			}
			//Finding the location of intersection points
			Matrix intersectionPointsCoordinates = Matrix.CreateZero(allPointsWithIntersection.Count, 2);
			for (int i = 0; i < allPointsWithIntersection.Count; i++)
			{
				int[] intpo = allPointsWithIntersection[i];
				Vector x1 = elementCoordinates.GetRow(intpo[0]);
				Vector x2 = elementCoordinates.GetRow(intpo[1]);
				var rel = Math.Abs(elementNodalLevelSet[intpo[0]] / elementNodalLevelSet[intpo[1]]);
				intersectionPointsCoordinates[i, 0] = x1[0] + (x2[0] - x1[0]) * rel / (1 + rel);
				intersectionPointsCoordinates[i, 1] = x1[1] + (x2[1] - x1[1]) * rel / (1 + rel);
			}
			//Add nodes of intersection points on nodes of element
			Matrix elementCoordinatesWithIntersection = elementCoordinates.AppendBottom(intersectionPointsCoordinates);
			Vector intersectionPointsLevelSet = Vector.CreateZero(numOfIntersectionPoints);
			Vector elementsNodalLevelSetWithIntersectionPoints = elementNodalLevelSet.Append(intersectionPointsLevelSet);
			Vector elementConnectionCircularWithIntersectionPoints = Vector.CreateFromArray(new double[] { 0, 1, 2, 3, 0 });
			// Αdd the intersection points on connection of elements with the right order!
			// For example:
			//
			//#3          |     #2
			//      .__#5.|_____.
			//      |     |     |
			//      |     |     |
			//      |     |     |
			//      |     |     |
			//      |     |     |
			//#0    .__#4.|_____. #1
			//            |
			//Final Result Of elementConnectionCircularWithIntersectionPoints= {0,4,1,2,5,3,0}.
			int ts = 0;
			int aa = 0;
			int bb = 0;
			int cc = 0;
			int dd = 0;
			double ip = 0;
			for (int i = 0; i < 4; i++)
			{
				if (nodalLevelSetCircular[i] * nodalLevelSetCircular[i + 1] < 0)
				{
					for (int j = 0; j < allPointsWithIntersection.Count; j++)
					{
						int[] intpo = allPointsWithIntersection[j];
						if (intpo[0] == elementNodesCircular[i])
						{
							aa = j;
						}
						if (intpo[1] == elementNodesCircular[i + 1])
						{
							bb = j;
						}
						if (intpo[0] == elementNodesCircular[i + 1])
						{
							cc = j;
						}
						if (intpo[1] == elementNodesCircular[i])
						{
							dd = j;
						}
					}
					if (aa == bb)
					{ ip = aa + 4; }
					else
					{ ip = cc + 4; }
					Vector elementConnectionCircularWithIntersectionPointsUp = elementConnectionCircularWithIntersectionPoints.GetSubvector(0, i + ts + 1);
					Vector elementConnectionCircularWithIntersectionPointsTail = elementConnectionCircularWithIntersectionPoints.GetSubvector(i + ts + 1, elementConnectionCircularWithIntersectionPoints.Length);
					Vector newEntry = Vector.CreateFromArray(new double[] { ip });
					Vector elementConnectionCircularWithIntersectionPointsUptail = elementConnectionCircularWithIntersectionPointsUp.Append(newEntry);
					Vector totalConnection = elementConnectionCircularWithIntersectionPointsUptail.Append(elementConnectionCircularWithIntersectionPointsTail);
					elementConnectionCircularWithIntersectionPoints = totalConnection;
					ts++;
				}
			}
			var tuple = new Tuple<Vector, Matrix, Vector>(elementConnectionCircularWithIntersectionPoints, elementCoordinatesWithIntersection, elementsNodalLevelSetWithIntersectionPoints);
			return tuple;
		}
		public (Matrix, int[]) CoordinatesNodesofXFEMpoints(Vector elementNodesCircularWithIntersectionPoints, Matrix elementCoordinatesWithIntersection, Vector elementNodalLevelSetWithIntersectionPoints)
		{   //This Method returns the Matrix elementCoordinatesWithIntersectionAndCentrePoints and int [] connectionCircularNoNegativeNodesWithIntersection.
			//For Example:
			//
			//        LevelSet                      Coordinates:
			//#3          |     #2                           #0(-1,-1)
			//      .__#5.|_____.                            #1(+1,-1)
			//      |     |     |                            #2(+1,+1)
			//      |  +  |  -  |                            #3(-1,+1)    
			//      |     |     |                            #4(0,-1)
			//      |     |     |                            #5(0,+1)
			//      |     |     |
			//#0    .__#4.|_____. #1
			//            |
			// We found the centre point of the positive part:
			//
			//        LevelSet                           Coordinates:
			//#3          |     #2                           #0(-1,-1)
			//      .__#5.|_____.                            #1(+1,-1)
			//      |     |     |                            #2(+1,+1)
			//      |  +  |  -  |                            #3(-1,+1)    
			//      | #6. |     |                            #4(0,-1)
			//      |     |     |                            #5(0,+1)
			//      |     |     |                            #6(-0.5,0)
			//#0    .__#4.|_____. #1
			//            |
			//The Return variables are:
			//elementCoordinatesWithIntersectionAndCentrePoints = {{-1,-1},{+1,-1},{+1,+1},{-1,+1},{0,-1},{0,+1},{-0.5,0}}
			//connectionCircularNoNegativeNodesWithIntersection = {0,4,5,3,0}
			List<int> listOfNoNegativeNodalLevelSet = new List<int>();
			int[] IntegerConnection = new int[elementNodesCircularWithIntersectionPoints.Length];
			for (int i = 0; i < elementNodesCircularWithIntersectionPoints.Length; i++)
			{
				IntegerConnection[i] = (int)elementNodesCircularWithIntersectionPoints[i];
			}
			Vector elementNodalLevelSetCircular = elementNodalLevelSetWithIntersectionPoints.GetSubvector(IntegerConnection);
			for (int i = 0; i < elementNodalLevelSetCircular.Length; i++)
			{
				if (elementNodalLevelSetCircular[i] >= 0)
				{
					listOfNoNegativeNodalLevelSet.Add(i);
				}
			}
			int[] noNegativeNodalLevelSet = new int[listOfNoNegativeNodalLevelSet.Count];
			for (int i = 0; i < listOfNoNegativeNodalLevelSet.Count; i++)
			{
				noNegativeNodalLevelSet[i] = listOfNoNegativeNodalLevelSet[i];
			}
			Vector noNegativeNodesCircularWithIntersection = elementNodesCircularWithIntersectionPoints.GetSubvector(noNegativeNodalLevelSet);
			//Ensure that the first and last element of vector are the same!
			if (noNegativeNodesCircularWithIntersection[0] != noNegativeNodesCircularWithIntersection[noNegativeNodesCircularWithIntersection.Length - 1])
			{
				Vector newentry = Vector.CreateFromArray(new double[] { noNegativeNodesCircularWithIntersection[0] });
				Vector noNegativeNodesCircularWithIntersectionWithUpperEnd = noNegativeNodesCircularWithIntersection.Append(newentry);
				noNegativeNodesCircularWithIntersection = noNegativeNodesCircularWithIntersectionWithUpperEnd;
			}
			int[] connectionCircularNoNegativeNodesWithIntersection = new int[noNegativeNodesCircularWithIntersection.Length];
			for (int i = 0; i < noNegativeNodesCircularWithIntersection.Length; i++)
			{
				connectionCircularNoNegativeNodesWithIntersection[i] = (int)noNegativeNodesCircularWithIntersection[i];
			}
			//correct matlab error
			int[] connectionNoNegativeNodesWithIntersection = new int[connectionCircularNoNegativeNodesWithIntersection.Length - 1];
			for (int i = 0; i < connectionNoNegativeNodesWithIntersection.Length; i++)
			{
				connectionNoNegativeNodesWithIntersection[i] = connectionCircularNoNegativeNodesWithIntersection[i];
			}
			Matrix elementCoordinatesOnNoNegativePart = elementCoordinatesWithIntersection.GetSubmatrix(connectionNoNegativeNodesWithIntersection, new int[] { 0, 1 });
			Matrix meanCoordinate = GeometryCalculations.Mean(elementCoordinatesOnNoNegativePart);
			Matrix elementCoordinatesWithIntersectionAndCentrePoints = elementCoordinatesWithIntersection.AppendBottom(meanCoordinate);
			return (elementCoordinatesWithIntersectionAndCentrePoints, connectionCircularNoNegativeNodesWithIntersection);
		}
		public int[,] ConnectionOfSubTriangles(int[] nodesofSubTriangles, Matrix coordsOfSubTriangles)
		{
			//For Example:
			//                  LevelSet                                     
			//                      |
			//      #3           #5 |               #2                             trianglesConnection={{0,4,6},
			//       .______________._______________.                                                   {4,5,6},
			//       |\            /|               |                                                   {5,3,6},
			//       | \          / |               |                                                   {3,0,6}}
			//       |  \   +    /  |        -      |
			//       |   \      /   |               |                              
			//       |    \    /    |               |
			//       |     \  /     |               |
			//       |      #6.     |               |
			//       |      / \     |               |
			//       |     /   \    |               |
			//       |    /     \   |               |
			//       |   /       \  |               |
			//       |  /         \ |               |
			//       | /           \|               |
			//     #0.____________#4.______________#1.
			//                      |
			//                      |
			int[,] trianglesConnection = new int[nodesofSubTriangles.Length - 1, 3];
			for (int i = 0; i < nodesofSubTriangles.Length - 1; i++)
			{
				trianglesConnection[i, 0] = nodesofSubTriangles[i];
				trianglesConnection[i, 1] = nodesofSubTriangles[i + 1];
				trianglesConnection[i, 2] = coordsOfSubTriangles.NumRows - 1;
			}
			return trianglesConnection;
		}
		public ElementSubtriangle2D [] CreateSubTrianglesOfElement(Matrix coordinatesOfElement, Matrix coordinatesOfTriangles, int[,] connectionOfTriangles)
		{
			double x01 = coordinatesOfElement[0, 0];
			double y01 = coordinatesOfElement[0, 1];
			double x02 = coordinatesOfElement[1, 0];
			double y02 = coordinatesOfElement[1, 1];
			double x03 = coordinatesOfElement[2, 0];
			double y03 = coordinatesOfElement[2, 1];
			double x04 = coordinatesOfElement[3, 0];
			double y04 = coordinatesOfElement[3, 1];
			//calculate the transformation matrix
			Matrix A = Matrix.CreateFromArray(new double[,] { { x01 ,y01, x01 * y01, 1 } ,
															   {x02 ,y02 ,x02*y02, 1 },
																{x03 ,y03 ,x03*y03, 1 },
															   {x04, y04 ,x04*y04 ,1 } });
			Matrix B = Matrix.CreateFromArray(new double[,] { { -1 ,- 1, 1, 1 },
															  { 1 ,-1, -1, 1} ,
															   { 1 ,1, 1, 1},
															   { -1, 1, -1, 1} });
			Matrix TM = A.Invert() * B;
			//calculate natural coordinates of triangle nodes
			Matrix tricoord = coordinatesOfTriangles;
			Matrix tricoordfirstcol = tricoord.GetSubmatrix(0, tricoord.NumRows, 0, 1);
			Matrix tricoordsecondcol = tricoord.GetSubmatrix(0, tricoord.NumRows, 1, 2);
			Matrix multElementsOfColumns = Matrix.CreateZero(tricoord.NumRows, 1);
			Matrix ones = Matrix.CreateZero(tricoord.NumRows, 1);
			for (int aa = 0; aa < tricoord.NumRows; aa++)
			{
				multElementsOfColumns[aa, 0] = tricoordfirstcol[aa, 0] * tricoordsecondcol[aa, 0];
				ones[aa, 0] = 1;
			}
			Matrix gcoords = tricoord.AppendRight(multElementsOfColumns).AppendRight(ones);
			Matrix TMfirstseccolumns = TM.GetSubmatrix(0, TM.NumRows, 0, 2);
			Matrix natcoords = gcoords * TMfirstseccolumns;
			var subtriangles = new ElementSubtriangle2D[connectionOfTriangles.GetLength(0)];
			for (int aa = 0; aa < connectionOfTriangles.GetLength(0); aa++)
			{
				int[] triangrow = new int[connectionOfTriangles.GetLength(1)];
				for (int bb = 0; bb < connectionOfTriangles.GetLength(1); bb++)
				{
					triangrow[bb] = connectionOfTriangles[aa, bb];
				}
				Matrix tcoor = natcoords.GetSubmatrix(triangrow, new int[] { 0, 1 });
				var triangle = new Triangle2D(tcoor.GetRow(0).CopyToArray(), tcoor.GetRow(1).CopyToArray(), tcoor.GetRow(2).CopyToArray());
				subtriangles[aa] = new ElementSubtriangle2D(triangle);				
			}
			return subtriangles;
		}
		public IElementSubcell[] FindConformingMesh(IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
		{
			Matrix coordinatesOfElement = Matrix.CreateFromArray(new double[,] {{ element.Nodes[0].X, element.Nodes[0].Y},
																		  {element.Nodes[1].X,element.Nodes[1].Y },
																		  {element.Nodes[2].X, element.Nodes[2].Y },
																		  {element.Nodes[3].X, element.Nodes[3].Y }});
			var (elementConnectionCircularWithIntersectionPoints, elementCoordinatesWithIntersection, elementsNodalLevelSetWithIntersectionPoints) = IntersectionPoints(coordinatesOfElement, ElementNodalLevelSetValues);
			//Coordinates of positive, intersection and centre points
			(Matrix elementCoordinatesWithIntersectionAndCentrePoints, int[] connectionCircularNoNegativeNodesWithIntersection) = CoordinatesNodesofXFEMpoints(elementConnectionCircularWithIntersectionPoints, elementCoordinatesWithIntersection, elementsNodalLevelSetWithIntersectionPoints);
			//Construct sub - triangular elements and compute their area
			var trianglesConnection = ConnectionOfSubTriangles(connectionCircularNoNegativeNodesWithIntersection, elementCoordinatesWithIntersectionAndCentrePoints);
			var subtriangles = CreateSubTrianglesOfElement(coordinatesOfElement, elementCoordinatesWithIntersectionAndCentrePoints, trianglesConnection);
			return subtriangles;
		}
	}
}
