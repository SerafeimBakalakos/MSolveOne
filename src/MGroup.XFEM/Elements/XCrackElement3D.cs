using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Discretization.Mesh;
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
using MGroup.XFEM.Materials.Duplicates;

namespace MGroup.XFEM.Elements
{
	public class XCrackElement3D : IXCrackElement
	{
		private readonly IElementGeometry elementGeometry;
		private readonly int id;
		private readonly int numStandardDofs;
		private readonly IDofType[][] standardDofTypes;

		private IDofType[][] allDofTypes;

		//private Dictionary<ClosedPhaseBoundary, IReadOnlyList<GaussPoint>> gaussPointsBoundary;
		private IReadOnlyList<GaussPoint> gaussPointsBulk;

		//TODO: this can be cached once for all standard elements of the same type
		private EvalInterpolation[] evalInterpolationsAtGPsVolume;

		/// <summary>
		/// In the same order as their corresponding <see cref="gaussPointsBulk"/>.
		/// </summary>
		private IContinuumMaterial[] materialsAtGPsBulk;

		private int numEnrichedDofs;

		public XCrackElement3D(int id, IReadOnlyList<XNode> nodes, IElementGeometry elementGeometry,
			IFractureMaterialField materialField, IIsoparametricInterpolation interpolation,
			IGaussPointExtrapolation gaussPointExtrapolation, IQuadrature standardQuadrature,
			CrackElementIntegrationStrategy bulkIntegration)
		{
			this.id = id;
			this.Nodes = nodes;
			this.elementGeometry = elementGeometry;

			this.Interpolation = interpolation;
			this.GaussPointExtrapolation = gaussPointExtrapolation;
			this.IntegrationStandard = standardQuadrature;
			this.IntegrationBulk = bulkIntegration;
			this.MaterialField = materialField;

			this.numStandardDofs = 3 * nodes.Count;
			this.standardDofTypes = new IDofType[nodes.Count][];
			for (int i = 0; i < nodes.Count; ++i)
			{
				standardDofTypes[i] = new IDofType[]
				{
					StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ
				};
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

		/// <summary>
		/// Area of the element in the global cartesian coordinate system
		/// </summary>
		public double CalcBulkSizeCartesian() => elementGeometry.CalcBulkSizeCartesian(Nodes);

		public double CalcBulkSizeNatural() => elementGeometry.CalcBulkSizeNatural();

		/// <summary>
		/// The displacement field derivatives are a 3x3 matrix: gradientU[i,j] = dui/dj where i is the vector component 
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
			var displacementGradient = Matrix.CreateZero(3, 3);

			// Standard contributions
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double ux = uStd[3 * nodeIdx];
				double uy = uStd[3 * nodeIdx + 1];
				double uz = uStd[3 * nodeIdx + 2];

				double dNdx = point.ShapeFunctionDerivatives[nodeIdx, 0];
				double dNdy = point.ShapeFunctionDerivatives[nodeIdx, 1];
				double dNdz = point.ShapeFunctionDerivatives[nodeIdx, 2];
				displacementGradient[0, 0] += dNdx * ux;
				displacementGradient[0, 1] += dNdy * ux;
				displacementGradient[0, 2] += dNdz * ux;
				displacementGradient[1, 0] += dNdx * uy;
				displacementGradient[1, 1] += dNdy * uy;
				displacementGradient[1, 2] += dNdz * uy;
				displacementGradient[2, 0] += dNdx * uz;
				displacementGradient[2, 1] += dNdy * uz;
				displacementGradient[2, 2] += dNdz * uz;
			}

			// Enriched contributions
			IReadOnlyDictionary<IEnrichmentFunction, EvaluatedFunction> evalEnrichments =
				EvaluateEnrichments(point);
			int dof = 0;
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double N = point.ShapeFunctions[nodeIdx];
				double dNdx = point.ShapeFunctionDerivatives[nodeIdx, 0];
				double dNdy = point.ShapeFunctionDerivatives[nodeIdx, 1];
				double dNdz = point.ShapeFunctionDerivatives[nodeIdx, 2];

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
					double Bz = dNdz * deltaPsi + N * gradPsi[2];

					double enrDisplacementX = uenr[dof++];
					double enrDisplacementY = uenr[dof++];
					double enrDisplacementZ = uenr[dof++];

					displacementGradient[0, 0] += Bx * enrDisplacementX;
					displacementGradient[0, 1] += By * enrDisplacementX;
					displacementGradient[0, 2] += Bz * enrDisplacementX;
					displacementGradient[1, 0] += Bx * enrDisplacementY;
					displacementGradient[1, 1] += By * enrDisplacementY;
					displacementGradient[1, 2] += Bz * enrDisplacementY;
					displacementGradient[2, 0] += Bx * enrDisplacementZ;
					displacementGradient[2, 1] += By * enrDisplacementZ;
					displacementGradient[2, 2] += Bz * enrDisplacementZ;
				}
			}

			return displacementGradient;
		}

		public IMatrix DampingMatrix(IElement element) => throw new NotImplementedException();

		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element) => allDofTypes;

		public override int GetHashCode() => ID.GetHashCode();

		public XPoint EvaluateFunctionsAt(double[] naturalPoint)
		{
			var result = new XPoint(3);
			result.Coordinates[CoordinateSystem.ElementNatural] = naturalPoint;
			result.Element = this;
			result.ShapeFunctions = Interpolation.EvaluateFunctionsAt(naturalPoint);
			return result;
		}

		public double[] FindCentroidCartesian() => Utilities.FindCentroidCartesian(3, Nodes);

		public (IReadOnlyList<GaussPoint>, IReadOnlyList<IContinuumMaterial>) GetMaterialsForBulkIntegration()
			=> (gaussPointsBulk, materialsAtGPsBulk);

		//TODO: This method should be moved to a base class. Enriched DOFs do not depend on the finite element, but are set globally.
		//      Also standard dofs per node can be provided be the element in another property 
		public void IdentifyDofs()
		{
			this.numEnrichedDofs = 0;
			foreach (XNode node in Nodes) this.numEnrichedDofs += 3 * node.EnrichmentFuncs.Count;

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
					var nodalDofs = new IDofType[3 + 3 * node.EnrichmentFuncs.Count];
					nodalDofs[0] = StructuralDof.TranslationX;
					nodalDofs[1] = StructuralDof.TranslationY;
					nodalDofs[2] = StructuralDof.TranslationZ;
					int j = 3;
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
			int numPointsBulk = gaussPointsBulk.Count;

			// Calculate and cache standard interpolation at bulk integration points.
			//TODO: for all standard elements of the same type, this should be cached only once
			this.evalInterpolationsAtGPsVolume = new EvalInterpolation[numPointsBulk];
			for (int i = 0; i < numPointsBulk; ++i)
			{
				evalInterpolationsAtGPsVolume[i] = Interpolation.EvaluateAllAt(Nodes, gaussPointsBulk[i].Coordinates);
			}

			// Create and cache materials at bulk integration points.
			this.materialsAtGPsBulk = new IContinuumMaterial[numPointsBulk];
			for (int i = 0; i < numPointsBulk; ++i)
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

				var xpoint = new XPoint(3);
				xpoint.Element = this;
				xpoint.ShapeFunctions = evalInterpolation.ShapeFunctions;
				xpoint.ShapeFunctionDerivatives = evalInterpolation.ShapeGradientsCartesian;
				xpoint.JacobianNaturalGlobal = evalInterpolation.Jacobian;

				double dV = evalInterpolation.Jacobian.DirectDeterminant;

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
				double dV = evalInterpolation.Jacobian.DirectDeterminant;

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

			var deformationMatrix = Matrix.CreateZero(6, numEnrichedDofs);
			int currentColumn = 0;
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double N = evalInterpolation.ShapeFunctions[nodeIdx];
				double dNdx = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 0];
				double dNdy = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 1];
				double dNdz = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 2];

				foreach (var enrichmentValuePair in Nodes[nodeIdx].EnrichmentFuncs)
				{
					IEnrichmentFunction enrichment = enrichmentValuePair.Key;
					double nodalPsi = enrichmentValuePair.Value;
					EvaluatedFunction evalEnrichment = enrichmentValues[enrichment];

					// Bi = enrN,i = N,i(x, y) * [psi(x, y) - psi(node)] + N(x,y) * psi,i(x,y), where i = x, y or z
					double dPsi = evalEnrichment.Value - nodalPsi;
					double Bx = dNdx * dPsi + N * evalEnrichment.CartesianDerivatives[0];
					double By = dNdy * dPsi + N * evalEnrichment.CartesianDerivatives[1];
					double Bz = dNdz * dPsi + N * evalEnrichment.CartesianDerivatives[2];

					// This depends on the convention: node major or enrichment major. 
					// The following is node major, since this convention is used throughout MSolve.
					int col0 = currentColumn++;
					int col1 = currentColumn++;
					int col2 = currentColumn++;

					deformationMatrix[0, col0] = Bx;
					deformationMatrix[1, col1] = By;
					deformationMatrix[2, col2] = Bz;

					deformationMatrix[3, col0] = By;
					deformationMatrix[3, col1] = Bx;

					deformationMatrix[4, col1] = Bz;
					deformationMatrix[4, col2] = By;

					deformationMatrix[5, col0] = Bz;
					deformationMatrix[5, col2] = Bx;
				}
			}
			Debug.Assert(currentColumn == numEnrichedDofs);
			return deformationMatrix;
		}

		/// <summary>
		/// Calculates the deformation matrix B. Dimensions = 6 x (3*numNodes).
		/// B is a linear transformation FROM the nodal values of the displacement field TO the the derivatives of
		/// the displacement field in respect to the cartesian axes (i.e. the stresses): 
		/// Bk = [dNk/dx 0 0; 0 dNk/dY 0; 0 0 dNk/dZ 0; dNk/dy dNk/dx 0; 0 dNk/dz dNk/dy; dNk/dz 0 dNk/dx] (6x3)
		/// </summary>
		/// <param name="evaluatedInterpolation">The shape function derivatives calculated at a specific 
		///     integration point</param>
		private Matrix CalcDeformationMatrixStandard(EvalInterpolation evalInterpolation)
		{
			var deformation = Matrix.CreateZero(6, numStandardDofs);
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				int col0 = 3 * nodeIdx;
				int col1 = 3 * nodeIdx + 1;
				int col2 = 3 * nodeIdx + 2;

				double dNdx = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 0];
				double dNdy = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 1];
				double dNdz = evalInterpolation.ShapeGradientsCartesian[nodeIdx, 2];

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
				stdDofIndices[3 * n] = totalDofCounter++;
				stdDofIndices[3 * n + 1] = totalDofCounter++;
				stdDofIndices[3 * n + 2] = totalDofCounter++;

				// Enr dofs
				for (int e = 0; e < Nodes[n].EnrichmentFuncs.Count; ++e)
				{
					for (int i = 0; i < 3; ++i)
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
