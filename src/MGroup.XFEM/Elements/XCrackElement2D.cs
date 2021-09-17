using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.ElementGeometry;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.ConformingMesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Interpolation.GaussPointExtrapolation;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.MSolve.Discretization.Loads;

namespace MGroup.XFEM.Elements
{
	public class XCrackElement2D : IXCrackElement
	{
		private readonly IElementGeometry elementGeometry;
		private readonly int id;
		private readonly int numStandardDofs;
		private readonly IDofType[][] standardDofTypes;

		private IDofType[][] allDofTypes;

		private Dictionary<ClosedPhaseBoundary, IReadOnlyList<GaussPoint>> gaussPointsBoundary;
		private IReadOnlyList<GaussPoint> gaussPointsBulk;

		//TODO: this can be cached once for all standard elements of the same type
		private EvalInterpolation[] evalInterpolationsAtGPsVolume;

		/// <summary>
		/// In the same order as their corresponding <see cref="gaussPointsBulk"/>.
		/// </summary>
		private IContinuumMaterial[] materialsAtGPsBulk;

		private int numEnrichedDofs;

		public XCrackElement2D(int id, IReadOnlyList<XNode> nodes, double thickness, IElementGeometry elementGeometry,
			IFractureMaterialField materialField, IIsoparametricInterpolation interpolation, 
			IGaussPointExtrapolation gaussPointExtrapolation, IQuadrature standardQuadrature,
			CrackElementIntegrationStrategy bulkIntegration)
		{
			this.id = id;
			this.Nodes = nodes;
			this.Thickness = thickness;
			this.elementGeometry = elementGeometry;

			this.Interpolation = interpolation;
			this.GaussPointExtrapolation = gaussPointExtrapolation;
			this.IntegrationStandard = standardQuadrature;
			this.IntegrationBulk = bulkIntegration;
			this.MaterialField = materialField;

			this.numStandardDofs = 2 * nodes.Count;
			this.standardDofTypes = new IDofType[nodes.Count][];
			for (int i = 0; i < nodes.Count; ++i)
			{
				standardDofTypes[i] = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };
			}

			int[] nodeIDs = nodes.Select(n => n.ID).ToArray();
			(this.Edges, this.Faces) = elementGeometry.FindEdgesFaces(nodeIDs);
		}

		public IReadOnlyList<GaussPoint> BulkIntegrationPoints => gaussPointsBulk;

		public CellType CellType => Interpolation.CellType;

		public IElementSubcell[] ConformingSubcells { get; set; }

		public IElementDofEnumerator DofEnumerator { get; set; } = new GenericDofEnumerator();

		public IElementType ElementType => this;

		public ElementEdge[] Edges { get; }

		public ElementFace[] Faces { get; }

		public IGaussPointExtrapolation GaussPointExtrapolation { get; }

		public int ID { get => id; set => throw new InvalidOperationException("ID is set at constructor."); }

		public IQuadrature IntegrationStandard { get; }

		public IBulkIntegration IntegrationBulk { get; set; }

		/// <summary>
		/// Common interpolation for standard and enriched nodes.
		/// </summary>
		public IIsoparametricInterpolation Interpolation { get; }

		public IFractureMaterialField MaterialField { get; }

		IReadOnlyList<INode> IElement.Nodes => Nodes;
		/// <summary>
		/// All nodes are enriched for now.
		/// </summary>
		public IReadOnlyList<XNode> Nodes { get; }

		public Dictionary<ICrack, IElementDiscontinuityInteraction> InteractingCracks { get; } 
			= new Dictionary<ICrack, IElementDiscontinuityInteraction>();

		public Dictionary<int, IElementDiscontinuityInteraction> InteractingDiscontinuities { get; }
			= new Dictionary<int, IElementDiscontinuityInteraction>();

		public int SubdomainID { get; set; } = int.MinValue;
		public void SetSubdomainID(int subdomainID) => SubdomainID = subdomainID;

		public double Thickness { get; }

		/// <summary>
		/// Area of the element in the global cartesian coordinate system
		/// </summary>
		public double CalcBulkSizeCartesian() => elementGeometry.CalcBulkSizeCartesian(Nodes);

		public double CalcBulkSizeNatural() => elementGeometry.CalcBulkSizeNatural();

		/// <summary>
		/// The displacement field derivatives are a 2x2 matrix: gradientU[i,j] = dui/dj where i is the vector component 
		/// and j is the coordinate, w.r.t which the differentiation is done. The differentation coordinates and the
		/// vector components refer to the global cartesian system. 
		/// WARNING: Do not call this method for points on a crack interface.
		/// </summary>
		public Matrix CalcDisplacementFieldGradient(XPoint point, Vector nodalDisplacements)
		{
			//TODO: There is a lot of duplication between this and the calculation of Bstd, Benr. I could abstract the common 
			//      code, however that will probably slow down then calculation of Bstd, Benr which is much more important.

			(int[] stdDofIndices, int[] enrDofIndices) = MapDofsFromStdEnrToNodeMajor();
			Vector uStd = nodalDisplacements.GetSubvector(stdDofIndices);
			Vector uenr = nodalDisplacements.GetSubvector(enrDofIndices);
			var displacementGradient = Matrix.CreateZero(2, 2);

			// Standard contributions
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double ux = uStd[2 * nodeIdx];
				double uy = uStd[2 * nodeIdx + 1];

				double dNdx = point.ShapeFunctionDerivatives[nodeIdx, 0];
				double dNdy = point.ShapeFunctionDerivatives[nodeIdx, 1];
				displacementGradient[0, 0] += dNdx * ux;
				displacementGradient[0, 1] += dNdy * ux;
				displacementGradient[1, 0] += dNdx * uy;
				displacementGradient[1, 1] += dNdy * uy;
			}

			// Enriched contributions.
			IReadOnlyDictionary<IEnrichmentFunction, EvaluatedFunction> evalEnrichments =
				EvaluateEnrichments(point);
			int dof = 0;
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double N = point.ShapeFunctions[nodeIdx];
				double dNdx = point.ShapeFunctionDerivatives[nodeIdx, 0];
				double dNdy = point.ShapeFunctionDerivatives[nodeIdx, 1];

				foreach (var enrichmentValuePair in Nodes[nodeIdx].EnrichmentFuncs)
				{
					IEnrichmentFunction enrichment = enrichmentValuePair.Key;
					double nodalPsi = enrichmentValuePair.Value;
					EvaluatedFunction evalEnrichment = evalEnrichments[enrichment];

					double psi = evalEnrichment.Value;
					double[] gradPsi = evalEnrichment.CartesianDerivatives;
					double deltaPsi = psi - nodalPsi;

					double Bx = dNdx * deltaPsi + N * gradPsi[0];
					double By = dNdy * deltaPsi + N * gradPsi[1];

					double enrDisplacementX = uenr[dof++];
					double enrDisplacementY = uenr[dof++];

					displacementGradient[0, 0] += Bx * enrDisplacementX;
					displacementGradient[0, 1] += By * enrDisplacementX;
					displacementGradient[1, 0] += Bx * enrDisplacementY;
					displacementGradient[1, 1] += By * enrDisplacementY;
				}
			}

			return displacementGradient;
		}

		public IMatrix DampingMatrix(IElement element) => throw new NotImplementedException();

		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element) => allDofTypes;

		public override int GetHashCode() => ID.GetHashCode();

		public XPoint EvaluateFunctionsAt(double[] naturalPoint)
		{
			var result = new XPoint(2);
			result.Coordinates[CoordinateSystem.ElementNatural] = naturalPoint;
			result.Element = this;
			result.ShapeFunctions = Interpolation.EvaluateFunctionsAt(naturalPoint);
			return result;
		}

		public double[] FindCentroidCartesian() => Utilities.FindCentroidCartesian(2, Nodes);

		public (IReadOnlyList<GaussPoint>, IReadOnlyList<IContinuumMaterial>) GetMaterialsForBulkIntegration()
			=> (gaussPointsBulk, materialsAtGPsBulk);

		//TODO: This method should be moved to a base class. Enriched DOFs do not depend on the finite element, but are set globally.
		//      Also standard dofs per node can be provided be the element in another property 
		public void IdentifyDofs()
		{
			this.numEnrichedDofs = 0;
			foreach (XNode node in Nodes) this.numEnrichedDofs += 2 * node.EnrichmentFuncs.Count;

			if (this.numEnrichedDofs == 0) allDofTypes = standardDofTypes;
			else
			{
				// The dof order in increasing frequency of change is: node, enrichment item, enrichment function, axis.
				// A similar convention should also hold for each enrichment item: enrichment function major, axis minor.
				// WARNING: The order here must match the order in JoinStiffnessesNodeMajor().
				this.allDofTypes = new IDofType[Nodes.Count][];
				for (int i = 0; i < Nodes.Count; ++i)
				{
					XNode node = Nodes[i];
					var nodalDofs = new IDofType[2 + 2 * node.EnrichmentFuncs.Count];
					nodalDofs[0] = StructuralDof.TranslationX;
					nodalDofs[1] = StructuralDof.TranslationY;
					int j = 2;
					foreach (EnrichmentItem enrichment in node.Enrichments)
					{
						foreach (IDofType dof in enrichment.EnrichedDofs)
						{
							nodalDofs[j++] = dof;
						}
					}
					this.allDofTypes[i] = nodalDofs;
				}
			}
		}

		public void IdentifyIntegrationPointsAndMaterials()
		{
			// Bulk integration
			this.gaussPointsBulk = IntegrationBulk.GenerateIntegrationPoints(this);
			int numPointsVolume = gaussPointsBulk.Count;

			// Calculate and cache standard interpolation at integration points.
			//TODO: for all standard elements of the same type, this should be cached only once
			this.evalInterpolationsAtGPsVolume = new EvalInterpolation[numPointsVolume];
			for (int i = 0; i < numPointsVolume; ++i)
			{
				evalInterpolationsAtGPsVolume[i] = Interpolation.EvaluateAllAt(Nodes, gaussPointsBulk[i].Coordinates);
			}

			// Create and cache materials at integration points.
			this.materialsAtGPsBulk = new IContinuumMaterial[numPointsVolume];
			for (int i = 0; i < numPointsVolume; ++i)
			{
				this.materialsAtGPsBulk[i] = MaterialField.FindMaterialAt(null);
			}
		}

		public IMatrix MassMatrix(IElement element) => throw new NotImplementedException();

		public IMatrix StiffnessMatrix(IElement element)
		{
			Matrix Kss = BuildStiffnessMatrixStandard();
			IMatrix Ktotal;
			if (numEnrichedDofs == 0) Ktotal = Kss;
			else
			{
				(Matrix Kee, Matrix Kse) = BuildStiffnessMatricesEnriched();
				Ktotal = JoinStiffnesses(Kss, Kee, Kse);
			}
			return Ktotal;
		}

		private (Matrix Kee, Matrix Kse) BuildStiffnessMatricesEnriched()
		{
			var Kse = Matrix.CreateZero(numStandardDofs, numEnrichedDofs);
			var Kee = Matrix.CreateZero(numEnrichedDofs, numEnrichedDofs);
			for (int i = 0; i < gaussPointsBulk.Count; ++i)
			{
				GaussPoint gaussPoint = gaussPointsBulk[i];
				EvalInterpolation evalInterpolation = evalInterpolationsAtGPsVolume[i];

				var xpoint = new XPoint(2);
				xpoint.Element = this;
				xpoint.ShapeFunctions = evalInterpolation.ShapeFunctions;
				xpoint.ShapeFunctionDerivatives = evalInterpolation.ShapeGradientsCartesian;
				xpoint.JacobianNaturalGlobal = evalInterpolation.Jacobian;

				double dV = evalInterpolation.Jacobian.DirectDeterminant * Thickness;

				// Material properties
				IMatrixView constitutive = materialsAtGPsBulk[i].ConstitutiveMatrix;

				// Deformation matrices: Bs = grad(Ns), Be = grad(Ne)
				Matrix Bstd = CalcDeformationMatrixStandard(evalInterpolation);
				Matrix Benr = CalculateDeformationMatrixEnriched(numEnrichedDofs, xpoint, evalInterpolation);

				// Contribution of this gauss point to the element stiffness matrices: 
				// Kee = SUM(Benr^T * C * Benr  *  dV*w), Kse = SUM(Bstd^T * C * Benr  *  dV*w)
				Matrix cBe = constitutive.MultiplyRight(Benr); // cache the result
				Matrix BeCBe = Benr.MultiplyRight(cBe, true, false);  // enriched-enriched part
				Kee.AxpyIntoThis(BeCBe, dV * gaussPoint.Weight);
				Matrix BsCBe = Bstd.MultiplyRight(cBe, true, false);  // enriched-standard part
				Kse.AxpyIntoThis(BsCBe, dV * gaussPoint.Weight);
			}
			return (Kee, Kse);
		}

		//TODO: A perhaps important optimization would be to use the std integration rule for Kss. However that creates the need 
		//      to track these integration points separately, which is cumbersome in non-linear problems. Perhaps it can be 
		//      delegated to an auxiliary class. In problems featuring both multiple phases and cracks, the whole idea may not 
		//      work at all.
		private Matrix BuildStiffnessMatrixStandard()
		{
			// If the element is has more than 1 phase, then I cannot use the standard quadrature, since the material is  
			// different on each phase.
			var Kss = Matrix.CreateZero(numStandardDofs, numStandardDofs);
			for (int i = 0; i < gaussPointsBulk.Count; ++i)
			{
				GaussPoint gaussPoint = gaussPointsBulk[i];
				EvalInterpolation evalInterpolation = evalInterpolationsAtGPsVolume[i];
				double dV = evalInterpolation.Jacobian.DirectDeterminant * Thickness;

				// Material properties
				IMatrixView constitutive = materialsAtGPsBulk[i].ConstitutiveMatrix;

				// Deformation matrix:  Bs = grad(Ns)
				Matrix deformation = CalcDeformationMatrixStandard(evalInterpolation);

				// Contribution of this gauss point to the element stiffness matrix: Kss = sum(Bs^T * c * Bs  *  dV*w)
				Matrix partial = deformation.ThisTransposeTimesOtherTimesThis(constitutive);
				Kss.AxpyIntoThis(partial, dV * gaussPoint.Weight);
			}
			return Kss;
		}

		private Matrix CalculateDeformationMatrixEnriched(int numEnrichedDofs, XPoint gaussPoint, 
			EvalInterpolation evalInterpolation)
		{
			Dictionary<IEnrichmentFunction, EvaluatedFunction> enrichmentValues = EvaluateEnrichments(gaussPoint);

			var deformationMatrix = Matrix.CreateZero(3, numEnrichedDofs);
			int currentColumn = 0;
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double N = evalInterpolation.ShapeFunctions[nodeIdx];
				double dNdx = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 0];
				double dNdy = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 1];

				foreach (var enrichmentValuePair in Nodes[nodeIdx].EnrichmentFuncs)
				{
					IEnrichmentFunction enrichment = enrichmentValuePair.Key;
					double nodalPsi = enrichmentValuePair.Value;
					EvaluatedFunction evalEnrichment = enrichmentValues[enrichment];
					
					// Bx = enrN,x = N,x(x, y) * [psi(x, y) - psi(node)] + N(x,y) * psi,x(x,y)
					// By = enrN,y = N,y(x, y) * [psi(x, y) - psi(node)] + N(x,y) * psi,y(x,y)
					double dPsi = evalEnrichment.Value - nodalPsi;
					double Bx = dNdx * dPsi + N * evalEnrichment.CartesianDerivatives[0];
					double By = dNdy * dPsi + N * evalEnrichment.CartesianDerivatives[1];

					// This depends on the convention: node major or enrichment major. 
					// The following is node major, since this convention is used throughout MSolve.
					int col1 = currentColumn++;
					int col2 = currentColumn++;
					deformationMatrix[0, col1] = Bx;
					deformationMatrix[1, col2] = By;
					deformationMatrix[2, col1] = By;
					deformationMatrix[2, col2] = Bx;
				}
			}
			Debug.Assert(currentColumn == numEnrichedDofs);
			return deformationMatrix;
		}

		/// <summary>
		/// Calculates the deformation matrix B. Dimensions = 3 x (2*numNodes).
		/// B is a linear transformation FROM the nodal values of the displacement field TO the the derivatives of
		/// the displacement field in respect to the cartesian axes (i.e. the stresses): {dU/dX} = [B] * {d} => 
		/// {u,x v,y u,y, v,x} = [... Bk ...] * {u1 v1 u2 v2 u3 v3 u4 v4}, where k = 1, ... nodesCount is a node and
		/// Bk = [dNk/dx 0; 0 dNk/dY; dNk/dy dNk/dx] (3x2)
		/// </summary>
		/// <param name="evaluatedInterpolation">The shape function derivatives calculated at a specific 
		///     integration point</param>
		private Matrix CalcDeformationMatrixStandard(EvalInterpolation evalInterpolation)
		{
			var deformation = Matrix.CreateZero(3, numStandardDofs);
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				int col0 = 2 * nodeIdx;
				int col1 = 2 * nodeIdx + 1;

				double dNdx = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 0];
				double dNdy = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 1];
				deformation[0, col0] = dNdx;
				deformation[1, col1] = dNdy;
				deformation[2, col0] = dNdy;
				deformation[2, col1] = dNdx;
			}
			return deformation;
		}

		//TODO: This can be used in all XFEM elements
		private Dictionary<IEnrichmentFunction, EvaluatedFunction> EvaluateEnrichments(XPoint gaussPoint)
		{
			var cachedEvalEnrichments = new Dictionary<IEnrichmentFunction, EvaluatedFunction>();
			foreach (XNode node in Nodes)
			{
				foreach (IEnrichmentFunction enrichment in node.EnrichmentFuncs.Keys)
				{
					// The enrichment function probably has been evaluated when processing a previous node. Avoid reevaluation.
					if (!cachedEvalEnrichments.TryGetValue(enrichment, out EvaluatedFunction evaluatedEnrichments))
					{
						evaluatedEnrichments = enrichment.EvaluateAllAt(gaussPoint);
						cachedEvalEnrichments[enrichment] = evaluatedEnrichments;
					}
				}
			}
			return cachedEvalEnrichments;
		}

		/// <summary>
		/// Copy the entries of Kss, Kse, Kee to the upper triangle of a total matrix for the element.
		/// </summary>
		private IMatrix JoinStiffnesses(Matrix Kss, Matrix Kee, Matrix Kse)
		{
			// Find the mapping from Kss, Kse, Kee to a total matrix for the element.
			(int[] stdDofIndices, int[] enrDofIndices) = MapDofsFromStdEnrToNodeMajor();
			var Ktotal = SymmetricMatrix.CreateZero(numStandardDofs + numEnrichedDofs);

			// Upper triangle of Kss
			for (int stdCol = 0; stdCol < numStandardDofs; ++stdCol)
			{
				int totalCol = stdDofIndices[stdCol];
				for (int stdRow = 0; stdRow <= stdCol; ++stdRow)
				{
					Ktotal[stdDofIndices[stdRow], totalCol] = Kss[stdRow, stdCol];
				}
			}

			for (int enrCol = 0; enrCol < numEnrichedDofs; ++enrCol)
			{
				int totalCol = enrDofIndices[enrCol];

				// Whole Kse
				for (int stdRow = 0; stdRow < numStandardDofs; ++stdRow)
				{
					Ktotal[stdDofIndices[stdRow], totalCol] = Kse[stdRow, enrCol];
				}

				// Upper triangle of Kee
				for (int enrRow = 0; enrRow <= enrCol; ++enrRow)
				{
					Ktotal[enrDofIndices[enrRow], totalCol] = Kee[enrRow, enrCol];
				}
			}

			return Ktotal;
		}

		private (int[] stdDofIndices, int[] enrDofIndices) MapDofsFromStdEnrToNodeMajor()
		{
			// WARNING: The order here must match the order assumed in other methods of this class

			// The dof order in increasing frequency of change is: node, enrichment item, enrichment function, axis.
			var stdDofIndices = new int[numStandardDofs];
			var enrDofIndices = new int[numEnrichedDofs];
			int enrDofCounter = 0, totalDofCounter = 0;
			for (int n = 0; n < Nodes.Count; ++n)
			{
				// Std dofs
				stdDofIndices[2 * n] = totalDofCounter++;
				stdDofIndices[2 * n + 1] = totalDofCounter++;

				// Enr dofs
				for (int e = 0; e < Nodes[n].EnrichmentFuncs.Count; ++e)
				{
					for (int i = 0; i < 2; ++i)
					{
						enrDofIndices[enrDofCounter++] = totalDofCounter++;
					}
				}
			}
			return (stdDofIndices, enrDofIndices);
		}

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
	}
}
