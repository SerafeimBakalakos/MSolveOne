using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.IsoXFEM.ElementStructuralStiffnessComputations;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Reduction;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.XFEM.Materials.Duplicates;

namespace MGroup.XFEM.IsoXFEM
{

   public class IsoXfemElement2D : IXFiniteElement
	{
        public enum PhaseElement
        {
            solidElement,
            voidElement,
            boundaryElement
        }
        public ElasticMaterial2D material;
        public GeometryProperties geometry;
        public List<XNode> nodesOfElement = new List<XNode>();
        public IEnumerable<XNode> NodesOfElement {get => nodesOfElement;}
		public int[] dofsOfElement;
		public readonly Matrix elasticityMatrix;
		private readonly double lengthOfElement;
		private readonly double heigthOfElement;
		public double areaOfElement;
		public Matrix stiffnessOfElement;
		private static IMatrixView defaultStiffness;
		public PhaseElement phaseElement;
		public Vector elementLevelSet;
		public Matrix coordinatesOfElement;
		private static IElementStructuralStiffnessComputation elementStructuralStiffnessComputation;

		public IsoXfemElement2D(int id, ElasticMaterial2D material, GeometryProperties geometry, IEnumerable<XNode> nodesOfElement)
		{
			ID = id >= 0 ? id : -id;
			this.geometry = geometry;
			this.material = material;
			this.nodesOfElement.AddRange(nodesOfElement);
			elasticityMatrix = Matrix.CreateFromArray(new double[,]
				{{ 1* (material.YoungModulus / (1-Math.Pow( material.PoissonRatio, 2))), material.PoissonRatio*(material.YoungModulus /(1-Math.Pow( material.PoissonRatio, 2))), 0},
				{ material.PoissonRatio*(material.YoungModulus / (1-Math.Pow( material.PoissonRatio, 2))),1*(material.YoungModulus / (1-Math.Pow(material.PoissonRatio, 2))),0},{ 0,0,((1 - material.PoissonRatio)/2)*(material.YoungModulus /(1-Math.Pow(material.PoissonRatio, 2)))} });
			lengthOfElement = geometry.length / geometry.numberOfElementsX;
			heigthOfElement = geometry.height / geometry.numberOfElementsY;
			areaOfElement = lengthOfElement * heigthOfElement;
			coordinatesOfElement = Matrix.CreateFromArray(new double[,] { { 0, 0 }, { lengthOfElement, 0 }, { lengthOfElement, heigthOfElement }, { 0, heigthOfElement } });
			elementStructuralStiffnessComputation = new ElementStructuralStiffnessFEMComputation();
			if (defaultStiffness == null)
			{
				defaultStiffness = elementStructuralStiffnessComputation.ElementStructuralStiffnessComputation(coordinatesOfElement, elasticityMatrix, geometry.thickness);
			}
			stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
				Interpolation = InterpolationQuad4.UniqueInstance;
				IntegrationStandard = GaussLegendre2D.GetQuadratureWithOrder(2, 2);
				IntegrationBulk = new IntegrationWithConformingSubtriangles2D(TriangleQuadratureSymmetricGaussian.Order2Points3);			
		}

		public void CalcStiffnessAndArea(/*Vector elementLevelSet*/)
		{
			if (elementLevelSet.Min() >= 0)
			{
				phaseElement = PhaseElement.solidElement;
				stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
				areaOfElement = lengthOfElement * heigthOfElement;
			}
			else if (elementLevelSet.Max() <= 0)
			{
				phaseElement = PhaseElement.voidElement;
				stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
				stiffnessOfElement.ScaleIntoThis(0.0001);
				areaOfElement = 0.00;
			}
			else
			{
				phaseElement = PhaseElement.boundaryElement;
				var integration = new XFEMIntegration();
				integration.MeshAndAreaOfSubElement(coordinatesOfElement, elementLevelSet);
				var elementStructuralStiffnessComputation = new ElementStructuralStiffnessXFEMComputation(integration.coordinatesOfBoundaryElement, integration.connectionOfBoundaryElement);
				areaOfElement = integration.areaBoundaryElement;
				var stiffness = elementStructuralStiffnessComputation.ElementStructuralStiffnessComputation(coordinatesOfElement, elasticityMatrix, geometry.thickness);
				stiffnessOfElement = stiffness;
			}
		}

		public IReadOnlyList<GaussPoint> BulkIntegrationPoints => throw new NotImplementedException();

		public IElementSubcell[] ConformingSubcells { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }

		public ElementEdge[] Edges => throw new NotImplementedException();

		public ElementFace[] Faces => throw new NotImplementedException();

		public IBulkIntegration IntegrationBulk { get; }

		public IQuadrature IntegrationStandard { get; }

		public IIsoparametricInterpolation Interpolation { get; } = InterpolationQuad4.UniqueInstance;

		public IReadOnlyList<XNode> Nodes => nodesOfElement;

		public Dictionary<int, IElementDiscontinuityInteraction> InteractingDiscontinuities => throw new NotImplementedException();

		public int ID { get; set; }

		public IElementType ElementType => this;

		IReadOnlyList<INode> IElement.Nodes => this.Nodes;

		public int SubdomainID { get; set; }

		public CellType CellType => CellType.Quad4;

		public IElementDofEnumerator DofEnumerator { get; set; } = new GenericDofEnumerator();

		public bool MaterialModified => throw new NotImplementedException();

		public void SetSubdomainID(int subdomainID) => throw new NotImplementedException();
		public double CalcBulkSizeCartesian()
		{
			if (elementLevelSet.Min() >= 0)
			{
				phaseElement = PhaseElement.solidElement;				
				areaOfElement = lengthOfElement * heigthOfElement;
			}
			else if (elementLevelSet.Max() <= 0)
			{
				phaseElement = PhaseElement.voidElement;				
				areaOfElement = 0.00;
			}
			else
			{
				phaseElement = PhaseElement.boundaryElement;
				var integration = new XFEMIntegration();
				integration.MeshAndAreaOfSubElement(coordinatesOfElement, elementLevelSet);
				//var elementStructuralStiffnessComputation = new ElementStructuralStiffnessXFEMComputation(integration.coordinatesOfBoundaryElement, integration.connectionOfBoundaryElement);
				areaOfElement = integration.areaBoundaryElement;
				//var stiffness = elementStructuralStiffnessComputation.ElementStructuralStiffnessComputation(coordinatesOfElement, elasticityMatrix, geometry.thickness);
				//stiffnessOfElement = stiffness;
			}
			return areaOfElement;
		}
		public double CalcBulkSizeNatural() => 4.00;
		public double[] FindCentroidCartesian() => throw new NotImplementedException();

		//for enriched elements
		public void IdentifyDofs() => throw new NotImplementedException();
		public void IdentifyIntegrationPointsAndMaterials()
		{

		}

		public IMatrix StiffnessMatrix(IElement element)
		{
			if (elementLevelSet.Min() >= 0)
			{
				phaseElement = PhaseElement.solidElement;
				stiffnessOfElement = defaultStiffness.CopyToFullMatrix();				
			}
			else if (elementLevelSet.Max() <= 0)
			{
				phaseElement = PhaseElement.voidElement;
				stiffnessOfElement = defaultStiffness.CopyToFullMatrix();
				stiffnessOfElement.ScaleIntoThis(0.0001);
			}
			else
			{
				phaseElement = PhaseElement.boundaryElement;
				var integration = new XFEMIntegration();
				integration.MeshAndAreaOfSubElement(coordinatesOfElement, elementLevelSet);
				var elementStructuralStiffnessComputation = new ElementStructuralStiffnessXFEMComputation(integration.coordinatesOfBoundaryElement, integration.connectionOfBoundaryElement);
				var stiffness = elementStructuralStiffnessComputation.ElementStructuralStiffnessComputation(coordinatesOfElement, elasticityMatrix, geometry.thickness);
				stiffnessOfElement = stiffness;
			}
			return stiffnessOfElement;
		}
		//Dynamic Problems
		public IMatrix MassMatrix(IElement element) => throw new NotImplementedException();
		public IMatrix DampingMatrix(IElement element) => throw new NotImplementedException();
		public void ResetMaterialModified() => throw new NotImplementedException();
		public Tuple<double[], double[]> CalculateStresses(IElement element, double[] localDisplacements) => throw new NotImplementedException();
		public double[] CalculateForces(IElement element) => throw new NotImplementedException();
		public double[] CalculateForcesForLogging(IElement element, double[] localDisplacements) => throw new NotImplementedException();
		public double[] CalculateAccelerationForces(IElement element, IList<MassAccelerationLoad> loads) => throw new NotImplementedException();
		public void SaveMaterialState() => throw new NotImplementedException();
		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element) => throw new NotImplementedException();
	}
}

