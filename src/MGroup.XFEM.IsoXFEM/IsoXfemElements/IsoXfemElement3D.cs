namespace MGroup.XFEM.IsoXFEM.IsoXfemElements
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.MSolve.Discretization.Mesh;
	using MGroup.XFEM.ElementGeometry;
	using MGroup.XFEM.Elements;
	using MGroup.XFEM.Entities;
	using MGroup.XFEM.Geometry;
	using MGroup.XFEM.Geometry.ConformingMesh;
	using MGroup.XFEM.Integration;
	using MGroup.XFEM.Integration.Quadratures;
	using MGroup.XFEM.Interpolation;
	using MGroup.XFEM.Materials.Duplicates;

	public class IsoXfemElement3D : IIsoXfemElement
	{
		private const int dim = 3;
		private readonly int numStandardDofs;
		private readonly GeometryProperties geometry;
		private readonly ElasticMaterial3D material;
		private readonly IElementGeometry elementGeometry;
		private IReadOnlyList<GaussPoint> gaussPointsBulk;
		private EvalInterpolation[] evalInterpolationsAtGPsBulk;
		private readonly List<XNode> nodesOfElement = new List<XNode>();
		private readonly IDofType[][] dofTypes;
		private readonly double lengthOfElement;
		private readonly double heigthOfElement;
		private static IMatrixView defaultStiffness;

		public IsoXfemElement3D(int id, ElasticMaterial3D material, GeometryProperties geometry, IEnumerable<XNode> nodesOfElement)
		{
			ID = id >= 0 ? id : -id;
			this.material = material;
			this.geometry = geometry;
			this.nodesOfElement.AddRange(nodesOfElement);
			lengthOfElement = geometry.length / geometry.NumberOfElementsX;
			heigthOfElement = geometry.height / geometry.NumberOfElementsY;
			Thickness = geometry.thickness / geometry.NumberOfElementsZ;
			SizeOfElement = lengthOfElement * heigthOfElement*Thickness;
			elementGeometry = new ElementHexa8Geometry();
			int[] nodeIDs = Nodes.Select(n => n.ID).ToArray();
			(this.Edges, this.Faces) = elementGeometry.FindEdgesFaces(nodeIDs);
			Interpolation = InterpolationHexa8.UniqueInstance;
			IntegrationStandard = GaussLegendre3D.GetQuadratureWithOrder(2, 2, 2);
			IntegrationBulk = new IntegrationWithConformingSubtetrahedra3D(TetrahedronQuadrature.Order2Points4);
			this.numStandardDofs = Nodes.Count * dim;
			dofTypes = new IDofType[Nodes.Count][];
			for (int i = 0; i < Nodes.Count; ++i)
			{
				dofTypes[i] = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY ,StructuralDof.TranslationZ};
			}
			if (defaultStiffness == null)
			{
				defaultStiffness = BuildStiffnessMatrix();
			}
			StiffnessOfElement = defaultStiffness.CopyToFullMatrix();
		}

		public double SizeOfElement { get ; set ; }
		public Matrix StiffnessOfElement { get ; set; }
		public IIsoXfemElement.Phase PhaseElement { get ; set ; }
		public Vector ElementLevelSet { get ; set ; }
		public double Thickness { get; }

		public IReadOnlyList<GaussPoint> BulkIntegrationPoints => gaussPointsBulk;

		public IElementSubcell[] ConformingSubcells { get ; set ; }

		public ElementEdge[] Edges { get; }

		public ElementFace[] Faces { get; }

		public IBulkIntegration IntegrationBulk { get; }

		public IQuadrature IntegrationStandard { get; }

		public IIsoparametricInterpolation Interpolation { get; } = InterpolationHexa8.UniqueInstance;

		public IReadOnlyList<XNode> Nodes => nodesOfElement;

		public Dictionary<int, IElementDiscontinuityInteraction> InteractingDiscontinuities { get; } = new Dictionary<int, IElementDiscontinuityInteraction>();
		public int ID { get ; set ; }

		public IElementType ElementType => this;

		public int SubdomainID { get; set; }

		public CellType CellType => CellType.Hexa8;

		public IElementDofEnumerator DofEnumerator { get ; set ; } = new GenericDofEnumerator();

		public bool MaterialModified { get; }

		IReadOnlyList<INode> IElement.Nodes => this.Nodes;

		public int[] IdOnAxis { get ; set ; }

		public double CalcBulkSizeCartesian() => elementGeometry.CalcBulkSizeCartesian(Nodes);
		public double CalcBulkSizeNatural() => 8.00;
		public double[] CalculateAccelerationForces(IElement element, IList<MassAccelerationLoad> loads) => throw new NotImplementedException();
		public double[] CalculateForces(IElement element) => throw new NotImplementedException();
		public double[] CalculateForcesForLogging(IElement element, double[] localDisplacements) => throw new NotImplementedException();
		public Tuple<double[], double[]> CalculateStresses(IElement element, double[] localDisplacements) => throw new NotImplementedException();
		public IMatrix DampingMatrix(IElement element) => throw new NotImplementedException();
		public void DefinePhaseOfElement()
		{
			if (ElementLevelSet.Min() >= 0)
			{
				PhaseElement = IIsoXfemElement.Phase.solidElement;
				SizeOfElement = CalcBulkSizeCartesian();
			}
			else if (ElementLevelSet.Max() <= 0)
			{
				PhaseElement = IIsoXfemElement.Phase.voidElement;
				SizeOfElement = 0.00;
			}
			else
			{
				PhaseElement = IIsoXfemElement.Phase.boundaryElement;
			}
		}
		public void SetIdOnAxis(params int[] idOnAxis) => IdOnAxis = idOnAxis;
		public double[] FindCentroidCartesian() => throw new NotImplementedException();
		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element) =>  dofTypes;
		public void IdentifyDofs() { }
		public void IdentifyIntegrationPointsAndMaterials()
		{
			this.gaussPointsBulk = IntegrationBulk.GenerateIntegrationPoints(this);
			int numPointsBulk = gaussPointsBulk.Count;

			// Calculate and cache standard interpolation at bulk integration points.
			//TODO: for all standard elements of the same type, this should be cached only once
			this.evalInterpolationsAtGPsBulk = new EvalInterpolation[numPointsBulk];
			for (int i = 0; i < numPointsBulk; ++i)
			{
				evalInterpolationsAtGPsBulk[i] = Interpolation.EvaluateAllAt(Nodes, gaussPointsBulk[i].Coordinates);
			}
		}
		
		public IMatrix MassMatrix(IElement element) => throw new NotImplementedException();
		public void ResetMaterialModified() => throw new NotImplementedException();
		public void SaveMaterialState() => throw new NotImplementedException();
		public void SetSubdomainID(int subdomainID) => SubdomainID = subdomainID;
		public IMatrix StiffnessMatrix(IElement element)
		{
			if (ElementLevelSet != null)
			{
				if (PhaseElement == IIsoXfemElement.Phase.solidElement)
				{
					StiffnessOfElement = defaultStiffness.CopyToFullMatrix();
				}
				else if (PhaseElement == IIsoXfemElement.Phase.voidElement)
				{
					StiffnessOfElement = defaultStiffness.CopyToFullMatrix();
					StiffnessOfElement.ScaleIntoThis(0.0001);
				}
				else
				{
					var stiffness = BuildStiffnessMatrix();
					StiffnessOfElement = stiffness.CopyToFullMatrix();
				}
				return StiffnessOfElement;
			}
			else
			{
				return StiffnessOfElement;
			}
		}
		public IMatrix BuildStiffnessMatrix()
		{
			IdentifyIntegrationPointsAndMaterials();
			var Kss = Matrix.CreateZero(numStandardDofs, numStandardDofs);
			for (int i = 0; i < gaussPointsBulk.Count; ++i)
			{
				GaussPoint gaussPoint = gaussPointsBulk[i];
				EvalInterpolation evalInterpolation = evalInterpolationsAtGPsBulk[i];
				double dV = evalInterpolation.Jacobian.DirectDeterminant * geometry.thickness;

				// Material properties
				IMatrixView constitutive = material.ConstitutiveMatrix;

				// Deformation matrix:  Bs = grad(Ns)
				Matrix deformation = CalcDeformationMatrixStandard(evalInterpolation);

				// Contribution of this gauss point to the element stiffness matrix: Kss = sum(Bs^T * c * Bs  *  dV*w)
				Matrix partial = deformation.ThisTransposeTimesOtherTimesThis(constitutive);
				Kss.AxpyIntoThis(partial, dV * gaussPoint.Weight);
			}
			return Kss;
		}
		private Matrix CalcDeformationMatrixStandard(EvalInterpolation evalInterpolation)
		{
			var deformation = Matrix.CreateZero(6, numStandardDofs);
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				int col0 = 3 * nodeIdx;
				int col1 = 3 * nodeIdx + 1;
				int col2 = 3 * nodeIdx + 2;

				double dNdx = evalInterpolation.ShapeGradientsGlobal[nodeIdx, 0];
				double dNdy = evalInterpolation.ShapeGradientsGlobal[nodeIdx, 1];
				double dNdz = evalInterpolation.ShapeGradientsGlobal[nodeIdx, 2];

				deformation[0, col0] = dNdx;
				deformation[1, col1] = dNdy;
				deformation[2, col2] = dNdz;

				deformation[3, col0] = dNdy;
				deformation[3, col1] = dNdx;

				deformation[4, col1] = dNdz;
				deformation[4, col2] = dNdy;

				deformation[5, col0] = dNdz;
				deformation[5, col2] = dNdx;
			}
			return deformation;
		}		
	}
	public static class IsoXfemElement3DExtensions
	{
		public static void RegisterInteractionWithLsm3D(this IXFiniteElement element,
		 IElementDiscontinuityInteraction interaction)
		{
			element.InteractingDiscontinuities[interaction.ParentGeometryID] = interaction;
		}
	}
}
