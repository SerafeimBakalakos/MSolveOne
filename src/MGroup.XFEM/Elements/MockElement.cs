using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.MSolve.Discretization.Loads;

//TODO: delete this class
namespace MGroup.XFEM.Elements
{
	public class MockElement : IXThermalElement
	{
		private readonly IElementGeometry elementGeometry;
		private readonly int dimension;
		private readonly int boundaryIntegrationOrder;
		private EvalInterpolation[] evalInterpolationsAtGPsBulk;
		private Dictionary<IPhaseBoundary, IReadOnlyList<GaussPoint>> gaussPointsBoundary;
		private Dictionary<IPhaseBoundary, IReadOnlyList<double[]>> gaussPointsBoundaryNormals;
		private IReadOnlyList<GaussPoint> gaussPointsBulk;
		private IPhase[] phasesAtGPsVolume;

		public MockElement(int id, CellType cellType, IReadOnlyList<XNode> nodes, int boundaryIntegrationOrder)
		{
			this.ID = id;
			this.CellType = cellType;
			this.Nodes = nodes;
			this.boundaryIntegrationOrder = boundaryIntegrationOrder;
			if (this.CellType == CellType.Tri3)
			{
				dimension = 2;
				elementGeometry = new ElementTri3Geometry();
				Interpolation = InterpolationTri3.UniqueInstance;
				IntegrationStandard = TriangleQuadratureSymmetricGaussian.Order2Points3;
				IntegrationBulk = new IntegrationWithConformingSubtriangles2D(TriangleQuadratureSymmetricGaussian.Order2Points3);

			}
			else if (this.CellType == CellType.Quad4)
			{
				dimension = 2;
				elementGeometry = new ElementQuad4Geometry();
				Interpolation = InterpolationQuad4.UniqueInstance;
				IntegrationStandard = GaussLegendre2D.GetQuadratureWithOrder(2, 2);
				IntegrationBulk = new IntegrationWithConformingSubtriangles2D(TriangleQuadratureSymmetricGaussian.Order2Points3);
			}
			else if (this.CellType == CellType.Tet4)
			{
				dimension = 3;
				elementGeometry = new ElementTet4Geometry();
				Interpolation = InterpolationTet4.UniqueInstance;
				IntegrationStandard = TetrahedronQuadrature.Order2Points4;
				IntegrationBulk = new IntegrationWithConformingSubtetrahedra3D(TetrahedronQuadrature.Order2Points4);
			}
			else if (this.CellType == CellType.Hexa8)
			{
				dimension = 3;
				elementGeometry = new ElementHexa8Geometry();
				Interpolation = InterpolationHexa8.UniqueInstance;
				IntegrationStandard = GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2);
				IntegrationBulk = new IntegrationWithConformingSubtetrahedra3D(TetrahedronQuadrature.Order2Points4);
			}

			int[] nodeIDs = nodes.Select(n => n.ID).ToArray();
			(Edges, Faces) = elementGeometry.FindEdgesFaces(nodeIDs);

		}

		public CellType CellType { get; }

		public IElementDofEnumerator DofEnumerator 
		{ 
			get => throw new NotImplementedException(); 
			set => throw new NotImplementedException(); 
		}

		public IElementType ElementType => throw new NotImplementedException();

		public int ID { get; set; }

		public IIsoparametricInterpolation Interpolation { get; }

		public IReadOnlyList<XNode> Nodes { get; }
		IReadOnlyList<INode> IElement.Nodes => Nodes;

		public HashSet<IPhase> Phases { get; } = new HashSet<IPhase>();

		public Dictionary<IPhaseBoundary, IElementDiscontinuityInteraction> PhaseIntersections { get; } 
			= new Dictionary<IPhaseBoundary, IElementDiscontinuityInteraction>();

		public Dictionary<int, IElementDiscontinuityInteraction> InteractingDiscontinuities { get; }
			= new Dictionary<int, IElementDiscontinuityInteraction>();

		public int SubdomainID { get; set; } = int.MinValue;
		public void SetSubdomainID(int subdomainID) => SubdomainID = subdomainID;


		public ElementEdge[] Edges { get; }

		public ElementFace[] Faces { get; }

		public IBulkIntegration IntegrationBulk { get; }

		public IElementSubcell[] ConformingSubcells { get; set; }

		public IQuadrature IntegrationStandard { get; }

		public IReadOnlyList<GaussPoint> BulkIntegrationPoints => gaussPointsBulk;

		public IReadOnlyList<GaussPoint> BoundaryIntegrationPoints
		{
			get
			{
				var allBoundaryPoints = new List<GaussPoint>();
				if (gaussPointsBoundary != null)
				{
					foreach (var points in gaussPointsBoundary.Values)
					{
						allBoundaryPoints.AddRange(points);
					}
				}
				return allBoundaryPoints;
			}
		}

		public IReadOnlyList<double[]> BoundaryIntegrationPointNormals
		{
			get
			{
				var allNormals = new List<double[]>();
				if (gaussPointsBoundaryNormals != null)
				{
					foreach (var normals in gaussPointsBoundaryNormals.Values)
					{
						allNormals.AddRange(normals);
					}
				}
				return allNormals;
			}
		}

		public IMatrix DampingMatrix(IElement element)
		{
			throw new NotImplementedException();
		}

		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element)
		{
			throw new NotImplementedException();
		}

		public IMatrix MassMatrix(IElement element)
		{
			throw new NotImplementedException();
		}

		public IMatrix StiffnessMatrix(IElement element)
		{
			throw new NotImplementedException();
		}

		public double CalcBulkSizeCartesian() => elementGeometry.CalcBulkSizeCartesian(Nodes);

		public double CalcBulkSizeNatural() => elementGeometry.CalcBulkSizeNatural();

		public void IdentifyDofs()
		{
		}

		public void IdentifyIntegrationPointsAndMaterials()
		{
			// Bulk integration
			this.gaussPointsBulk = IntegrationBulk.GenerateIntegrationPoints(this);
			int numPointsBulk = gaussPointsBulk.Count;

			// Calculate and cache standard interpolation at bulk integration points.
			//TODO: for all standard elements of the same type, this should be cached only once
			this.evalInterpolationsAtGPsBulk = new EvalInterpolation[numPointsBulk];
			for (int i = 0; i < numPointsBulk; ++i)
			{
				evalInterpolationsAtGPsBulk[i] = Interpolation.EvaluateAllAt(Nodes, gaussPointsBulk[i].Coordinates);
			}

			// Find and cache the phase at bulk integration points.
			this.phasesAtGPsVolume = new IPhase[numPointsBulk];
			Debug.Assert(Phases.Count != 0);
			if (Phases.Count == 1)
			{
				IPhase commonPhase = Phases.First();
				for (int i = 0; i < numPointsBulk; ++i) this.phasesAtGPsVolume[i] = commonPhase;
			}
			else
			{
				for (int i = 0; i < numPointsBulk; ++i)
				{
					XPoint point = new XPoint(dimension);
					point.Element = this;
					point.Coordinates[CoordinateSystem.ElementNatural] = gaussPointsBulk[i].Coordinates;
					point.ShapeFunctions = evalInterpolationsAtGPsBulk[i].ShapeFunctions;
					point.ShapeFunctionDerivativesGlobal = evalInterpolationsAtGPsBulk[i].ShapeGradientsGlobal;
					IPhase phase = this.FindPhaseAt(point);
					point.PhaseID = phase.ID;
					this.phasesAtGPsVolume[i] = phase;
				}
			}

			// Create and cache materials at boundary integration points.
			this.gaussPointsBoundary = new Dictionary<IPhaseBoundary, IReadOnlyList<GaussPoint>>();
			this.gaussPointsBoundaryNormals = new Dictionary<IPhaseBoundary, IReadOnlyList<double[]>>();
			foreach (var boundaryIntersectionPair in PhaseIntersections)
			{
				IPhaseBoundary boundary = boundaryIntersectionPair.Key;
				IElementDiscontinuityInteraction intersection = boundaryIntersectionPair.Value;

				IReadOnlyList<GaussPoint> gaussPoints = intersection.GetBoundaryIntegrationPoints(boundaryIntegrationOrder);
				IReadOnlyList<double[]> gaussPointsNormals =
					intersection.GetNormalsAtBoundaryIntegrationPoints(boundaryIntegrationOrder);
				int numGaussPoints = gaussPoints.Count;

				gaussPointsBoundary[boundary] = gaussPoints;
				gaussPointsBoundaryNormals[boundary] = gaussPointsNormals;
			}
		}

		public XPoint EvaluateFunctionsAt(double[] naturalPoint)
		{
			throw new NotImplementedException();
		}

		public Dictionary<IPhaseBoundary, (IReadOnlyList<GaussPoint>, IReadOnlyList<ThermalInterfaceMaterial>)> 
			GetMaterialsForBoundaryIntegration()
		{
			throw new NotImplementedException();
		}

		public (IReadOnlyList<GaussPoint>, IReadOnlyList<ThermalMaterial>) GetMaterialsForBulkIntegration()
		{
			throw new NotImplementedException();
		}

		public double[] FindCentroidCartesian() => Utilities.FindCentroidCartesian(dimension, Nodes);

		#region non linear
		public bool MaterialModified => throw new NotImplementedException();


		public void ResetMaterialModified()
		{
			throw new NotImplementedException();
		}

		public Tuple<double[], double[]> CalculateStresses(IElement element, double[] localDisplacements)
		{
			throw new NotImplementedException();
		}

		public double[] CalculateForces(IElement element)
		{
			throw new NotImplementedException();
		}

		public double[] CalculateForcesForLogging(IElement element, double[] localDisplacements)
		{
			throw new NotImplementedException();
		}

		public double[] CalculateAccelerationForces(IElement element, IList<MassAccelerationLoad> loads)
		{
			throw new NotImplementedException();
		}

		public void SaveMaterialState()
		{
			throw new NotImplementedException();
		}
		#endregion
		public class Factory : IXElementFactory<MockElement>
		{
			private readonly int boundaryIntegrationOrder;

			public Factory(int boundaryIntegrationOrder = 2)
			{
				this.boundaryIntegrationOrder = boundaryIntegrationOrder;
			}

			public MockElement CreateElement(int id, CellType cellType, IReadOnlyList<XNode> nodes)
			{
				return new MockElement(id, cellType, nodes, boundaryIntegrationOrder);
			}
		}
	}
}
