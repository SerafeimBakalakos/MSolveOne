using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
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

//TODO: Now this element will work for either cohesive interfaces or coherent ones. A better design would be to work with both 
//      simultaneously. Each interface would determine its type. Then the unnecessary geometric (e.g. boundary GPs) and mechanic 
//      data can be avoided for only some interfaces.
namespace MGroup.XFEM.Elements
{
	public class XThermalElement2D : IXThermalElement
	{
		private readonly int boundaryIntegrationOrder;
		private readonly bool cohesiveInterfaces;
		private readonly IElementGeometry elementGeometry;
		private readonly int id;
		private readonly int numStandardDofs;
		private readonly IDofType[][] standardDofTypes;

		private IDofType[][] allDofTypes;

		private Dictionary<IPhaseBoundary, IReadOnlyList<GaussPoint>> gaussPointsBoundary;
		private IReadOnlyList<GaussPoint> gaussPointsBulk;

		//TODO: this can be cached once for all standard elements of the same type
		private EvalInterpolation[] evalInterpolationsAtGPsBulk;

		/// <summary>
		/// In the same order as their corresponding <see cref="gaussPointsBoundary"/>.
		/// </summary>
		private Dictionary<IPhaseBoundary, ThermalInterfaceMaterial[]> materialsAtGPsBoundary;

		/// <summary>
		/// In the same order as their corresponding <see cref="gaussPointsBulk"/>.
		/// </summary>
		private ThermalMaterial[] materialsAtGPsBulk;

		private int numEnrichedDofs;

		/// <summary>
		/// In the same order as their corresponding <see cref="gaussPointsBulk"/>.
		/// </summary>
		private IPhase[] phasesAtGPsBulk;

		public XThermalElement2D(int id, IReadOnlyList<XNode> nodes, double thickness, IElementGeometry elementGeometry,
			IThermalMaterialField materialField, IIsoparametricInterpolation interpolation, 
			IGaussPointExtrapolation gaussPointExtrapolation, IQuadrature standardQuadrature, 
			IBulkIntegration bulkIntegration, int boundaryIntegrationOrder, bool cohesiveInterfaces)
		{
			this.id = id;
			this.Nodes = nodes;
			this.Thickness = thickness;
			this.elementGeometry = elementGeometry;

			this.Interpolation = interpolation;
			this.GaussPointExtrapolation = gaussPointExtrapolation;
			this.IntegrationStandard = standardQuadrature;
			this.IntegrationBulk = bulkIntegration;
			this.boundaryIntegrationOrder = boundaryIntegrationOrder;
			this.cohesiveInterfaces = cohesiveInterfaces;
			this.MaterialField = materialField;

			this.numStandardDofs = nodes.Count;
			this.standardDofTypes = new IDofType[nodes.Count][];
			for (int i = 0; i < nodes.Count; ++i) this.standardDofTypes[i] = new IDofType[] { ThermalDof.Temperature };

			int[] nodeIDs = nodes.Select(n => n.ID).ToArray();
			(this.Edges, this.Faces) = elementGeometry.FindEdgesFaces(nodeIDs);
		}

		public IReadOnlyList<GaussPoint> BulkIntegrationPoints => gaussPointsBulk;

		public IReadOnlyList<GaussPoint> BoundaryIntegrationPoints
		{
			get
			{
				var allBoundaryPoints = new List<GaussPoint>();
				foreach (var points in gaussPointsBoundary.Values)
				{
					allBoundaryPoints.AddRange(points);
				}
				return allBoundaryPoints;
			}
		}

		public IReadOnlyList<double[]> BoundaryIntegrationPointNormals => new double[0][];

		public CellType CellType => Interpolation.CellType;

		public IElementSubcell[] ConformingSubcells { get; set; }

		public IElementDofEnumerator DofEnumerator { get; set; } = new GenericDofEnumerator();

		public IElementType ElementType => this;

		public ElementEdge[] Edges { get; }

		public ElementFace[] Faces { get; }

		public IGaussPointExtrapolation GaussPointExtrapolation { get; }

		public int ID { get => id; set => throw new InvalidOperationException("ID is set at constructor."); }

		//TODO: This should not always be used for Kss. E.g. it doesn't work for bimaterial interface.
		public IQuadrature IntegrationStandard { get; }

		public IBulkIntegration IntegrationBulk { get; set; }

		/// <summary>
		/// Common interpolation for standard and enriched nodes.
		/// </summary>
		public IIsoparametricInterpolation Interpolation { get; }

		public IThermalMaterialField MaterialField { get; }

		IReadOnlyList<INode> IElement.Nodes => Nodes;
		/// <summary>
		/// All nodes are enriched for now.
		/// </summary>
		public IReadOnlyList<XNode> Nodes { get; }

		public Dictionary<int, IElementDiscontinuityInteraction> InteractingDiscontinuities { get; }
			= new Dictionary<int, IElementDiscontinuityInteraction>();

		public HashSet<IPhase> Phases { get; } = new HashSet<IPhase>();

		public Dictionary<IPhaseBoundary, IElementDiscontinuityInteraction> PhaseIntersections { get; }
			= new Dictionary<IPhaseBoundary, IElementDiscontinuityInteraction>();

		public int SubdomainID { get; set; } = int.MinValue;
		public void SetSubdomainID(int subdomainID) => SubdomainID = subdomainID;

		public double Thickness { get; }

		public double CalcBulkSizeCartesian() => elementGeometry.CalcBulkSizeCartesian(Nodes);

		public double CalcBulkSizeNatural() => elementGeometry.CalcBulkSizeNatural();

		public IMatrix DampingMatrix(IElement element) => throw new NotImplementedException();

		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element) => allDofTypes;

		public XPoint EvaluateFunctionsAt(double[] naturalPoint)
		{
			var result = new XPoint(2);
			result.Coordinates[CoordinateSystem.ElementNatural] = naturalPoint;
			result.Element = this;
			result.ShapeFunctions = Interpolation.EvaluateFunctionsAt(naturalPoint);
			return result;
		}

		public double[] FindCentroidCartesian() => Utilities.FindCentroidCartesian(2, Nodes);


		public Dictionary<IPhaseBoundary, (IReadOnlyList<GaussPoint>, IReadOnlyList<ThermalInterfaceMaterial>)>
			GetMaterialsForBoundaryIntegration()
		{
			var result = new Dictionary<IPhaseBoundary, (IReadOnlyList<GaussPoint>, IReadOnlyList<ThermalInterfaceMaterial>)>();
			foreach (ClosedPhaseBoundary boundary in gaussPointsBoundary.Keys)
			{
				result[boundary] = (gaussPointsBoundary[boundary], materialsAtGPsBoundary[boundary]);
			}
			return result;
		}

		public (IReadOnlyList<GaussPoint>, IReadOnlyList<ThermalMaterial>) GetMaterialsForBulkIntegration()
			=> (gaussPointsBulk, materialsAtGPsBulk);

		public void IdentifyDofs()
		{
			this.numEnrichedDofs = 0;
			foreach (XNode node in Nodes) this.numEnrichedDofs += node.EnrichmentFuncs.Count;

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
					var nodalDofs = new IDofType[1 + node.EnrichmentFuncs.Count];
					nodalDofs[0] = ThermalDof.Temperature;
					int j = 1;
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
			this.evalInterpolationsAtGPsBulk = new EvalInterpolation[numPointsBulk];
			for (int i = 0; i < numPointsBulk; ++i)
			{
				evalInterpolationsAtGPsBulk[i] = Interpolation.EvaluateAllAt(Nodes, gaussPointsBulk[i].Coordinates);
			}

			// Find and cache the phase at bulk integration points.
			this.phasesAtGPsBulk = new IPhase[numPointsBulk];
			Debug.Assert(Phases.Count != 0);
			if (Phases.Count == 1)
			{
				IPhase commonPhase = Phases.First();
				for (int i = 0; i < numPointsBulk; ++i) this.phasesAtGPsBulk[i] = commonPhase;
			}
			else
			{
				for (int i = 0; i < numPointsBulk; ++i)
				{
					XPoint point = new XPoint(2);
					point.Element = this;
					point.Coordinates[CoordinateSystem.ElementNatural] = gaussPointsBulk[i].Coordinates;
					point.ShapeFunctions = evalInterpolationsAtGPsBulk[i].ShapeFunctions;
					point.ShapeFunctionDerivatives = evalInterpolationsAtGPsBulk[i].ShapeGradientsCartesian;
					IPhase phase = this.FindPhaseAt(point);
					point.PhaseID = phase.ID;
					this.phasesAtGPsBulk[i] = phase;
				}
			}

			// Create and cache materials at bulk integration points.
			this.materialsAtGPsBulk = new ThermalMaterial[numPointsBulk];
			for (int i = 0; i < numPointsBulk; ++i)
			{
				this.materialsAtGPsBulk[i] = MaterialField.FindMaterialAt(this.phasesAtGPsBulk[i]);
			}

			// Create and cache materials at boundary integration points, if they are needed.
			if (!cohesiveInterfaces) return;
			this.gaussPointsBoundary = new Dictionary<IPhaseBoundary, IReadOnlyList<GaussPoint>>();
			this.materialsAtGPsBoundary = new Dictionary<IPhaseBoundary, ThermalInterfaceMaterial[]>();
			foreach (var boundaryIntersectionPair in PhaseIntersections)
			{
				IPhaseBoundary boundary = boundaryIntersectionPair.Key;
				IElementDiscontinuityInteraction intersection = boundaryIntersectionPair.Value;

				IReadOnlyList<GaussPoint> gaussPoints = intersection.GetBoundaryIntegrationPoints(boundaryIntegrationOrder);
				int numGaussPoints = gaussPoints.Count;
				var materials = new ThermalInterfaceMaterial[numGaussPoints];

				//TODO: perhaps I should have one for each Gauss point
				ThermalInterfaceMaterial material = MaterialField.FindInterfaceMaterialAt(boundary);
				for (int i = 0; i < numGaussPoints; ++i) materials[i] = material;

				gaussPointsBoundary[boundary] = gaussPoints;
				materialsAtGPsBoundary[boundary] = materials;
			}
		}

		public IMatrix MassMatrix(IElement element) => throw new NotImplementedException();

		public IMatrix StiffnessMatrix(IElement element)
		{
			Matrix Kss = BuildConductivityMatrixStandard();
			IMatrix Ktotal;
			if (numEnrichedDofs == 0) Ktotal = Kss;
			else
			{
				(Matrix Kee, Matrix Kse) = BuildConductivityMatricesEnriched();
				if (cohesiveInterfaces && (PhaseIntersections.Count > 0))
				{
					Matrix Kii = BuildConductivityMatrixBoundary();
					Kee.AddIntoThis(Kii);
				}
				Ktotal = JoinStiffnesses(Kss, Kee, Kse);
			}
			return Ktotal;
		}

		private (Matrix Kee, Matrix Kse) BuildConductivityMatricesEnriched()
		{
			var Kse = Matrix.CreateZero(numStandardDofs, numEnrichedDofs);
			var Kee = Matrix.CreateZero(numEnrichedDofs, numEnrichedDofs);
			for (int i = 0; i < gaussPointsBulk.Count; ++i)
			{
				GaussPoint gaussPoint = gaussPointsBulk[i];
				EvalInterpolation evalInterpolation = evalInterpolationsAtGPsBulk[i];

				var gaussPointAlt = new XPoint(2);
				gaussPointAlt.Element = this;
				gaussPointAlt.Coordinates[CoordinateSystem.ElementNatural] = gaussPoint.Coordinates;
				gaussPointAlt.ShapeFunctions = evalInterpolation.ShapeFunctions;
				gaussPointAlt.ShapeFunctionDerivatives = evalInterpolationsAtGPsBulk[i].ShapeGradientsCartesian;

				double dV = evalInterpolation.Jacobian.DirectDeterminant * Thickness;

				// Material properties
				double conductivity = materialsAtGPsBulk[i].ThermalConductivity;

				// Deformation matrices: Bs = grad(Ns), Be = grad(Ne)
				Matrix Bstd = CalcDeformationMatrixStandard(evalInterpolation);
				Matrix Benr = CalculateDeformationMatrixEnriched(numEnrichedDofs, gaussPointAlt, evalInterpolation);

				// Contribution of this gauss point to the element stiffness matrices: 
				// Kee = SUM(Benr^T * c * Benr  *  dV*w), Kse = SUM(Bstd^T * c * Benr  *  dV*w)
				Matrix cBe = conductivity * Benr; // cache the result
				Matrix BeCBe = Benr.MultiplyRight(cBe, true, false);  // enriched-enriched part
				Kee.AxpyIntoThis(BeCBe, dV * gaussPoint.Weight);
				Matrix BsCBe = Bstd.MultiplyRight(cBe, true, false);  // enriched-standard part
				Kse.AxpyIntoThis(BsCBe, dV * gaussPoint.Weight);
			}
			return (Kee, Kse);
		}

		#region delete
		//private Matrix BuildConductivityMatrixBoundaryOLD()
		//{
		//    var Kii = Matrix.CreateZero(numEnrichedDofs, numEnrichedDofs);
		//    foreach (var boundaryGaussPointsPair in gaussPointsBoundary)
		//    {
		//        PhaseBoundary boundary = boundaryGaussPointsPair.Key;
		//        IReadOnlyList<GaussPoint> gaussPoints = boundaryGaussPointsPair.Value;
		//        ThermalInterfaceMaterial[] materials = materialsAtGPsBoundary[boundary];

		//        // Kii = sum(conductivity * jumpCoeff^2 * N^T * N * weight * thickness)
		//        double phaseJumpCoeff = boundary.Enrichment.PhaseJumpCoefficient;
		//        double commonCoeff = phaseJumpCoeff * phaseJumpCoeff * Thickness;
		//        for (int i = 0; i < gaussPoints.Count; ++i)
		//        {
		//            GaussPoint gaussPoint = gaussPoints[i];
		//            double interfaceConductivity = materials[i].InterfaceConductivity;
		//            double scale = commonCoeff * interfaceConductivity * gaussPoint.Weight;

		//            Vector N = CalculateEnrichedShapeFunctionVector(gaussPoint, boundary);
		//            Matrix NtN = N.TensorProduct(N);
		//            Kii.AxpyIntoThis(NtN, scale);
		//        }
		//    }
		//    return Kii;
		//}
		#endregion

		private Matrix BuildConductivityMatrixBoundary()
		{
			var Kii = Matrix.CreateZero(numEnrichedDofs, numEnrichedDofs);
			foreach (var boundaryGaussPointsPair in gaussPointsBoundary)
			{
				IPhaseBoundary boundary = boundaryGaussPointsPair.Key;
				IReadOnlyList<GaussPoint> gaussPoints = boundaryGaussPointsPair.Value;
				ThermalInterfaceMaterial[] materials = materialsAtGPsBoundary[boundary];

				// Kii = sum(conductivity * N^T * N * weight * thickness)
				for (int i = 0; i < gaussPoints.Count; ++i)
				{
					GaussPoint gaussPoint = gaussPoints[i];
					double interfaceConductivity = materials[i].InterfaceConductivity;
					double scale = Thickness * interfaceConductivity * gaussPoint.Weight;

					Vector N = CalculateEnrichedShapeFunctionVector(gaussPoint.Coordinates, boundary);
					Matrix NtN = N.TensorProduct(N);
					Kii.AxpyIntoThis(NtN, scale);
				}
			}
			return Kii;
		}

		private Matrix BuildConductivityMatrixStandard()
		{
			// If the element is has more than 1 phase, then I cannot use the standard quadrature, since the material is  
			// different on each phase.
			var Kss = Matrix.CreateZero(numStandardDofs, numStandardDofs);
			for (int i = 0; i < gaussPointsBulk.Count; ++i)
			{
				GaussPoint gaussPoint = gaussPointsBulk[i];
				EvalInterpolation evalInterpolation = evalInterpolationsAtGPsBulk[i];
				double dV = evalInterpolation.Jacobian.DirectDeterminant * Thickness;
				//TODO: The thickness is constant per element in FEM, but what about XFEM? Different materials within the same 
				//      element are possible. Yeah but the thickness is a geometric porperty, rather than a material one.

				// Material properties
				double conductivity = materialsAtGPsBulk[i].ThermalConductivity;

				// Deformation matrix:  Bs = grad(Ns)
				Matrix deformation = CalcDeformationMatrixStandard(evalInterpolation);

				// Contribution of this gauss point to the element stiffness matrix: Kss = sum(Bs^T * c * Bs  *  dV*w)
				Matrix partial = deformation.MultiplyRight(deformation, true);
				Kss.AxpyIntoThis(partial, conductivity * dV * gaussPoint.Weight);
			}
			return Kss;
		}

		private Matrix CalculateDeformationMatrixEnriched(int numEnrichedDofs, XPoint gaussPoint,
			EvalInterpolation evaluatedInterpolation)
		{
			// For each node and with all derivatives w.r.t. cartesian coordinates, the enrichment derivatives 
			// are: Bx = enrN,x = N,x(x,y) * [psi(x,y) - psi(node)] + N(x,y) * psi,x(x,y), where psi is the  
			// enrichment function. However in this formulation of multiphase XFEM, only piecewise constant enrichments
			// are used. Therefore always psi,x = 0.

			var uniqueEnrichments = new Dictionary<IEnrichmentFunction, double>();

			var deformationMatrix = Matrix.CreateZero(2, numEnrichedDofs);
			int currentColumn = 0;
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
			{
				double dNdx = evaluatedInterpolation.ShapeGradientsCartesian[nodeIdx, 0];
				double dNdy = evaluatedInterpolation.ShapeGradientsCartesian[nodeIdx, 1];

				foreach (var enrichmentValuePair in Nodes[nodeIdx].EnrichmentFuncs)
				{
					IEnrichmentFunction enrichment = enrichmentValuePair.Key;
					double nodalPsi = enrichmentValuePair.Value;

					// The enrichment function probably has been evaluated when processing a previous node. Avoid reevaluation.
					double psi;
					if (!(uniqueEnrichments.TryGetValue(enrichment, out psi)))
					{
						psi = enrichment.EvaluateAt(gaussPoint);
						uniqueEnrichments[enrichment] = psi;
					}

					// Bx = enrN,x = N,x(x, y) * [psi(x, y) - psi(node)]
					// By = enrN,y = N,y(x, y) * [psi(x, y) - psi(node)]
					double dPsi = psi - nodalPsi;
					double Bx = dNdx * dPsi;
					double By = dNdy * dPsi;

					// This depends on the convention: node major or enrichment major. 
					// The following is node major, since this convention is used throughout MSolve.
					int col = currentColumn++;
					deformationMatrix[0, col] = Bx;
					deformationMatrix[1, col] = By;
				}
			}
			Debug.Assert(currentColumn == numEnrichedDofs);
			return deformationMatrix;
		}

		private Matrix CalcDeformationMatrixStandard(EvalInterpolation evalInterpolation)
		{
			// gradT = [ T,x ] = [ sum(Ni,x) * Ti ] = [ ... Ni,x ... ] * [ ... ]
			//         [ T,y ]   [ sum(Ni,y) * Ti ]   [ ... Ni,y ... ]   [  Ti ]
			//                                                           [ ... ]

			// The ones stored are [ N1,x N2,x N3,x ... ]. Therefore they need transposing
			//                     [ N1,y N2,y N3,y ... ]
			return evalInterpolation.ShapeGradientsCartesian.Transpose();
		}

		#region delete
		///// <summary>
		///// The contour integral along a phase boundary is calculated for the enriched dofs that were applied due to that 
		///// boundary. For example, if there are 2 boundaries and all 3 nodes of the element are enriched due to them, then the 6
		///// enriched dofs are [node1Boundary1, node1Boundary2, node2Boundary1, node2Boundary2, node3Boundary1, node3Boundary2].
		///// When integrating along boundary 1 we will compute N^T*N, where N(6x1) = [N1 0 N2 0 N3 0]. If we integrate along
		///// boundary 2, then N(6x1) = [0 N1 0 N2 0 N3]. 
		///// 
		///// Therefore when integrating along a specific boundary, then for every enriched dof of each node i, we need to find 
		///// if the enrichment was applied due to that boundary. If yes, the corresponding index of the total shape function 
		///// array gets the value Ni. Otherwise it remains 0. 
		///// 
		///// The whole thing also takes care of a) blending enrichments due to boundaries in other elements, 
		///// b) rare cases where one or more nodes were not enriched like the rest, because their nodal support was almost 
		///// entirely in one of the two regions.
		///// </summary>
		//private Vector CalculateEnrichedShapeFunctionVectorOLD(NaturalPoint gaussPoint, PhaseBoundary boundary)
		//{
		//    //TODO: Optimize this: The mapping should be done once per enrichment ane reused for all Gauss points.
		//    //      See an attempt at MapEnrichedDofIndicesToNodeIndices().

		//    Vector totalShapeFunctions = Vector.CreateZero(numEnrichedDofs);
		//    double[] N = InterpolationStandard.EvaluateFunctionsAt(gaussPoint);
		//    int idx = 0;
		//    for (int n = 0; n < Nodes.Count; ++n)
		//    {
		//        XNode node = Nodes[n];
		//        //TODO: VERY FRAGILE CODE. This order of enrichments was used to determine the order of enriched dofs in 
		//        //      another method. It works as of the time of writing, but this dependency must be removed. Perhaps use a 
		//        //      DofTable.
		//        foreach (IEnrichment enrichment in node.Enrichments.Keys) 
		//        {
		//            if (enrichment.IsAppliedDueTo(boundary)) totalShapeFunctions[idx] = N[n];
		//            ++idx; // always move to the next index in the total shape function array
		//        }
		//    }
		//    return totalShapeFunctions;
		//}
		#endregion

		/// <summary>
		/// The contour integral along a phase boundary is calculated for the enriched dofs that were applied due to that 
		/// boundary. For example, if there are 2 boundaries and all 3 nodes of the element are enriched due to them, then the 6
		/// enriched dofs are [node1Boundary1, node1Boundary2, node2Boundary1, node2Boundary2, node3Boundary1, node3Boundary2].
		/// When integrating along boundary 1 we will compute N^T*N, where N(6x1) = [N1 0 N2 0 N3 0]. If we integrate along
		/// boundary 2, then N(6x1) = [0 N1 0 N2 0 N3]. 
		/// 
		/// Therefore when integrating along a specific boundary, then for every enriched dof of each node i, we need to find 
		/// if the enrichment was applied due to that boundary. If yes, the corresponding index of the total shape function 
		/// array gets the value Ni. Otherwise it remains 0. 
		/// 
		/// The whole thing also takes care of a) blending enrichments due to boundaries in other elements, 
		/// b) rare cases where one or more nodes were not enriched like the rest, because their nodal support was almost 
		/// entirely in one of the two regions.
		/// </summary>
		private Vector CalculateEnrichedShapeFunctionVector(double[] gaussPoint, IPhaseBoundary boundary)
		{
			//TODO: Optimize this: The mapping should be done once per enrichment and reused for all Gauss points of this 
			//      boundary. That would only work if the jump is independent of the gauss point though, which is not always true.
			//      See an attempt at MapEnrichedDofIndicesToNodeIndices().
			
			double[] N = Interpolation.EvaluateFunctionsAt(gaussPoint);
			var point = new XPoint(2);
			point.Coordinates[CoordinateSystem.ElementNatural] = gaussPoint;
			point.ShapeFunctions = N;

			Vector totalShapeFunctions = Vector.CreateZero(numEnrichedDofs);
			int idx = 0;
			for (int n = 0; n < Nodes.Count; ++n)
			{
				XNode node = Nodes[n];
				//TODO: VERY FRAGILE CODE. This order of enrichments was used to determine the order of enriched dofs in 
				//      another method. It works as of the time of writing, but this dependency must be removed. Perhaps use a 
				//      DofTable.
				foreach (IEnrichmentFunction enrichment in node.EnrichmentFuncs.Keys)
				{
					double phaseJump = enrichment.EvaluateJumpAcross(boundary, point);
					totalShapeFunctions[idx] = phaseJump * N[n];
					++idx; // always move to the next index in the total shape function array
				}
			}
			return totalShapeFunctions;
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
				stdDofIndices[n] = totalDofCounter;
				totalDofCounter += 1;

				// Enr dofs
				for (int e = 0; e < Nodes[n].EnrichmentFuncs.Count; ++e)
				{
					enrDofIndices[enrDofCounter++] = totalDofCounter++;
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
