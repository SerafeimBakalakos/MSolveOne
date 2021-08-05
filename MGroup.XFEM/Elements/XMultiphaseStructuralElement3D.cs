using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.LinearAlgebra;
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

//TODO: A lot of duplication between this, the 2D version and thermal elements.
namespace MGroup.XFEM.Elements
{
    public class XMultiphaseStructuralElement3D : IXStructuralMultiphaseElement
    {
        private const int dim = 3;

        private readonly int boundaryIntegrationOrder;
        private readonly bool cohesiveInterfaces;
        private readonly IElementGeometry elementGeometry;
        private readonly int id;
        private readonly int numStandardDofs;
        private readonly IDofType[][] standardDofTypes;

        private IDofType[][] allDofTypes;

        private Dictionary<IPhaseBoundary, IReadOnlyList<GaussPoint>> gaussPointsBoundary;
        private Dictionary<IPhaseBoundary, IReadOnlyList<double[]>> gaussPointsBoundaryNormals;
        private IReadOnlyList<GaussPoint> gaussPointsBulk;

        //TODO: this can be cached once for all standard elements of the same type
        private EvalInterpolation[] evalInterpolationsAtGPsBulk;

        /// <summary>
        /// In the same order as their corresponding <see cref="gaussPointsBoundary"/>.
        /// </summary>
        private Dictionary<IPhaseBoundary, CohesiveInterfaceMaterial[]> materialsAtGPsBoundary;

        /// <summary>
        /// In the same order as their corresponding <see cref="gaussPointsBulk"/>.
        /// </summary>
        private IContinuumMaterial[] materialsAtGPsBulk;

        private int numEnrichedDofs;

        /// <summary>
        /// In the same order as their corresponding <see cref="gaussPointsBulk"/>.
        /// </summary>
        private IPhase[] phasesAtGPsVolume;

        public XMultiphaseStructuralElement3D(int id, IReadOnlyList<XNode> nodes, IElementGeometry elementGeometry,
            IStructuralMaterialField materialField, IIsoparametricInterpolation interpolation,
            IGaussPointExtrapolation gaussPointExtrapolation, IQuadrature standardQuadrature,
            IBulkIntegration bulkIntegration, int boundaryIntegrationOrder, bool cohesiveInterfaces)
        {
            this.id = id;
            this.Nodes = nodes;
            this.elementGeometry = elementGeometry;

            this.Interpolation = interpolation;
            this.GaussPointExtrapolation = gaussPointExtrapolation;
            this.IntegrationStandard = standardQuadrature;
            this.IntegrationBulk = bulkIntegration;
            this.boundaryIntegrationOrder = boundaryIntegrationOrder;
            this.cohesiveInterfaces = cohesiveInterfaces;
            this.MaterialField = materialField;

            this.numStandardDofs = dim * nodes.Count;
            this.standardDofTypes = new IDofType[nodes.Count][];
            for (int i = 0; i < nodes.Count; ++i)
            {
                this.standardDofTypes[i] = new IDofType[]
                {
                    StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ
                };
            }

            int[] nodeIDs = nodes.Select(n => n.ID).ToArray();
            (this.Edges, this.Faces) = elementGeometry.FindEdgesFaces(nodeIDs);
        }

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

        public IStructuralMaterialField MaterialField { get; }

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

        ISubdomain IElement.Subdomain => this.Subdomain;
        public XSubdomain Subdomain { get; set; }

        public double CalcBulkSizeCartesian() => elementGeometry.CalcBulkSizeCartesian(Nodes);

        public double CalcBulkSizeNatural() => elementGeometry.CalcBulkSizeNatural();

        public IMatrix DampingMatrix(IElement element) => throw new NotImplementedException();

        public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element) => allDofTypes;

        public XPoint EvaluateFunctionsAt(double[] naturalPoint)
        {
            var result = new XPoint(dim);
            result.Coordinates[CoordinateSystem.ElementNatural] = naturalPoint;
            result.Element = this;
            result.ShapeFunctions = Interpolation.EvaluateFunctionsAt(naturalPoint);
            return result;
        }

        public double[] FindCentroidCartesian() => Utilities.FindCentroidCartesian(dim, Nodes);


        public Dictionary<IPhaseBoundary, (IReadOnlyList<GaussPoint>, IReadOnlyList<CohesiveInterfaceMaterial>)>
            GetMaterialsForBoundaryIntegration()
        {
            var result = new Dictionary<IPhaseBoundary, (IReadOnlyList<GaussPoint>, IReadOnlyList<CohesiveInterfaceMaterial>)>();
            foreach (ClosedPhaseBoundary boundary in gaussPointsBoundary.Keys)
            {
                result[boundary] = (gaussPointsBoundary[boundary], materialsAtGPsBoundary[boundary]);
            }
            return result;
        }

        public (IReadOnlyList<GaussPoint>, IReadOnlyList<IContinuumMaterial>) GetMaterialsForBulkIntegration()
            => (gaussPointsBulk, materialsAtGPsBulk);

        public void IdentifyDofs()
        {
            this.numEnrichedDofs = 0;
            foreach (XNode node in Nodes) this.numEnrichedDofs += dim * node.EnrichmentFuncs.Count;

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
                    var nodalDofs = new IDofType[dim + dim * node.EnrichmentFuncs.Count];
                    nodalDofs[0] = StructuralDof.TranslationX;
                    nodalDofs[1] = StructuralDof.TranslationY;
                    nodalDofs[2] = StructuralDof.TranslationZ;
                    int j = dim;
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
                    XPoint point = new XPoint(dim);
                    point.Element = this;
                    point.Coordinates[CoordinateSystem.ElementNatural] = gaussPointsBulk[i].Coordinates;
                    point.ShapeFunctions = evalInterpolationsAtGPsBulk[i].ShapeFunctions;
                    point.ShapeFunctionDerivatives = evalInterpolationsAtGPsBulk[i].ShapeGradientsCartesian;
                    IPhase phase = this.FindPhaseAt(point);
                    point.PhaseID = phase.ID;
                    this.phasesAtGPsVolume[i] = phase;
                }
            }

            // Create and cache materials at bulk integration points.
            this.materialsAtGPsBulk = new IContinuumMaterial[numPointsBulk];
            for (int i = 0; i < numPointsBulk; ++i)
            {
                this.materialsAtGPsBulk[i] = MaterialField.FindMaterialAt(this.phasesAtGPsVolume[i]);
            }

            // Create and cache materials at boundary integration points.
            if (!cohesiveInterfaces) return;
            this.gaussPointsBoundary = new Dictionary<IPhaseBoundary, IReadOnlyList<GaussPoint>>();
            this.gaussPointsBoundaryNormals = new Dictionary<IPhaseBoundary, IReadOnlyList<double[]>>();
            this.materialsAtGPsBoundary = new Dictionary<IPhaseBoundary, CohesiveInterfaceMaterial[]>();
            foreach (var boundaryIntersectionPair in PhaseIntersections)
            {
                IPhaseBoundary boundary = boundaryIntersectionPair.Key;
                IElementDiscontinuityInteraction intersection = boundaryIntersectionPair.Value;

                IReadOnlyList<GaussPoint> gaussPoints = intersection.GetBoundaryIntegrationPoints(boundaryIntegrationOrder);
                IReadOnlyList<double[]> gaussPointsNormals =
                    intersection.GetNormalsAtBoundaryIntegrationPoints(boundaryIntegrationOrder);
                int numGaussPoints = gaussPoints.Count;
                var materials = new CohesiveInterfaceMaterial[numGaussPoints];

                //TODO: perhaps I should have one for each Gauss point
                CohesiveInterfaceMaterial material = MaterialField.FindInterfaceMaterialAt(boundary);
                for (int i = 0; i < numGaussPoints; ++i) materials[i] = material;

                gaussPointsBoundary[boundary] = gaussPoints;
                gaussPointsBoundaryNormals[boundary] = gaussPointsNormals;
                materialsAtGPsBoundary[boundary] = materials;
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
                if (cohesiveInterfaces && (PhaseIntersections.Count > 0))
                {
                    Matrix Kii = BuildStiffnessMatrixBoundary();
                    Kee.AddIntoThis(Kii);
                }
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
                EvalInterpolation evalInterpolation = evalInterpolationsAtGPsBulk[i];

                var gaussPointAlt = new XPoint(dim);
                gaussPointAlt.Element = this;
                gaussPointAlt.ShapeFunctions = evalInterpolation.ShapeFunctions;
                gaussPointAlt.ShapeFunctionDerivatives = evalInterpolationsAtGPsBulk[i].ShapeGradientsCartesian;

                double dV = evalInterpolation.Jacobian.DirectDeterminant;

                // Material properties
                IMatrixView constitutive = materialsAtGPsBulk[i].ConstitutiveMatrix;

                // Deformation matrices: Bs = grad(Ns), Be = grad(Ne)
                Matrix Bstd = CalcDeformationMatrixStandard(evalInterpolation);
                Matrix Benr = CalculateDeformationMatrixEnriched(numEnrichedDofs, gaussPointAlt, evalInterpolation);

                // Contribution of this gauss point to the element stiffness matrices: 
                // Kee = SUM(Benr^T * c * Benr  *  dV*w), Kse = SUM(Bstd^T * c * Benr  *  dV*w)
                Matrix cBe = constitutive.MultiplyRight(Benr); // cache the result
                Matrix BeCBe = Benr.MultiplyRight(cBe, true, false);  // enriched-enriched part
                Kee.AxpyIntoThis(BeCBe, dV * gaussPoint.Weight);
                Matrix BsCBe = Bstd.MultiplyRight(cBe, true, false);  // enriched-standard part
                Kse.AxpyIntoThis(BsCBe, dV * gaussPoint.Weight);
            }
            return (Kee, Kse);
        }

        private Matrix BuildStiffnessMatrixBoundary()
        {
            var Kii = Matrix.CreateZero(numEnrichedDofs, numEnrichedDofs);
            //foreach (var boundaryGaussPointsPair in gaussPointsBoundary)
            //{
            //    IPhaseBoundary boundary = boundaryGaussPointsPair.Key;
            //    IReadOnlyList<GaussPoint> gaussPoints = boundaryGaussPointsPair.Value;
            //    IReadOnlyList<double[]> normalVectorsAtGPs = gaussPointsBoundaryNormals[boundary];
            //    CohesiveInterfaceMaterial[] materials = materialsAtGPsBoundary[boundary];

            //    // Kii = sum(N^T * T * N * weight * thickness)
            //    for (int i = 0; i < gaussPoints.Count; ++i)
            //    {
            //        GaussPoint gaussPoint = gaussPoints[i];
            //        double dA = gaussPoint.Weight;

            //        IMatrix localT = materials[i].ConstitutiveMatrix;
            //        double[] normalVector = normalVectorsAtGPs[i];
            //        Matrix T = RotateInterfaceCohesiveTensor(localT, normalVector);
            //        Matrix N = CalculateEnrichedShapeFunctionMatrix(gaussPoint.Coordinates, boundary);
            //        Matrix partialKii = N.ThisTransposeTimesOtherTimesThis(T);
            //        Kii.AxpyIntoThis(partialKii, dA);
            //    }
            //}
            return Kii;
        }

        private Matrix BuildStiffnessMatrixStandard()
        {
            // If the element has more than 1 phase, then I cannot use the standard quadrature, since the material is  
            // different on each phase.
            var Kss = Matrix.CreateZero(numStandardDofs, numStandardDofs);
            for (int i = 0; i < gaussPointsBulk.Count; ++i)
            {
                GaussPoint gaussPoint = gaussPointsBulk[i];
                EvalInterpolation evalInterpolation = evalInterpolationsAtGPsBulk[i];
                double dV = evalInterpolation.Jacobian.DirectDeterminant;

                // Material properties
                IMatrixView constitutive = materialsAtGPsBulk[i].ConstitutiveMatrix;

                // Deformation matrix: Bs = grad(Ns)
                Matrix deformation = CalcDeformationMatrixStandard(evalInterpolation);

                // Contribution of this gauss point to the element stiffness matrix: Kss = sum(Bs^T * c * Bs  *  dV*w)
                Matrix partial = deformation.ThisTransposeTimesOtherTimesThis(constitutive);
                Kss.AxpyIntoThis(partial, dV * gaussPoint.Weight);
            }
            return Kss;
        }

        private Matrix CalculateDeformationMatrixEnriched(int numEnrichedDofs, XPoint gaussPoint,
            EvalInterpolation evaluatedInterpolation)
        {
            Dictionary<IEnrichmentFunction, EvaluatedFunction> enrichmentValues = EvaluateEnrichments(gaussPoint);

            var deformationMatrix = Matrix.CreateZero(6, numEnrichedDofs);
            int currentColumn = 0;
            for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
            {
                double N = evaluatedInterpolation.ShapeFunctions[nodeIdx];
                double dNdx = evaluatedInterpolation.ShapeGradientsCartesian[nodeIdx, 0];
                double dNdy = evaluatedInterpolation.ShapeGradientsCartesian[nodeIdx, 1];
                double dNdz = evaluatedInterpolation.ShapeGradientsCartesian[nodeIdx, 2];

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


        
        private Matrix CalculateEnrichedShapeFunctionMatrix(double[] gaussPoint, IPhaseBoundary boundary)
        {
            throw new NotImplementedException();
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
                stdDofIndices[dim * n] = totalDofCounter++;
                stdDofIndices[dim * n + 1] = totalDofCounter++;
                stdDofIndices[dim * n + 2] = totalDofCounter++;

                // Enr dofs
                for (int e = 0; e < Nodes[n].EnrichmentFuncs.Count; ++e)
                {
                    for (int i = 0; i < dim; ++i)
                    {
                        enrDofIndices[enrDofCounter++] = totalDofCounter++;
                    }
                }
            }
            return (stdDofIndices, enrDofIndices);
        }

        private Matrix RotateInterfaceCohesiveTensor(IMatrix localTensor, double[] normalVector)
        {
            throw new NotImplementedException();
        }
    }
}
