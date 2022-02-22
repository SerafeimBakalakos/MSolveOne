namespace MGroup.XFEM.IsoXFEM.SolidOnlyTriangulator
{
	using System;
	using System.Collections.Generic;
	using System.Text;

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
		 
		public Vector CalcNodalLevelSetOfSubTet4(ElementEdge edge, double[] centreOfFace, double[] centroid, IXFiniteElement element)
		{
			Vector nodalLevelSetSubTet4 = Vector.CreateZero(4);
			nodalLevelSetSubTet4[0] = ElementNodalLevelSetValues[edge.NodeIDs[0]];
			nodalLevelSetSubTet4[1] = ElementNodalLevelSetValues[edge.NodeIDs[1]];
			var interpolationCentreOfFace = element.Interpolation.EvaluateAllAt(element.Nodes, centreOfFace);
			var shapeFunctionValuesCentreOfFace = interpolationCentreOfFace.ShapeFunctions;
			var interpolationCentroid = element.Interpolation.EvaluateAllAt(element.Nodes, centroid);
			var shapeFunctionValuesCentroid = interpolationCentroid.ShapeFunctions;
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
				if ((int)subtetrahedra3D[i].PhaseOfSubTet4 == 0)
				{
					solidSubtetrahedra3D.Add(subtetrahedra3D[i]);
				}
				else if ((int)subtetrahedra3D[i].PhaseOfSubTet4 == 2)
				{
					boundarySubtetrahedra3D.Add(subtetrahedra3D[i]);
				}
			}
			return (solidSubtetrahedra3D, boundarySubtetrahedra3D);
		}

		public IElementSubcell[] FindConformingMesh(IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance) => throw new NotImplementedException();
		//public IElementSubcell[] FindConformingMesh(IXFiniteElement element, IEnumerable<IElementDiscontinuityInteraction> intersections, IMeshTolerance meshTolerance)
		//{

		//}

	}
}
