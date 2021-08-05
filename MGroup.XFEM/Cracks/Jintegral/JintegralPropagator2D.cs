using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.PropagationCriteria;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Interpolation;
using MGroup.XFEM.Materials;

namespace MGroup.XFEM.Cracks.Jintegral
{
    /// <summary>
    /// Calculates the direction and length of the next crack segment. For now it only works with homogeneous materials.
    /// </summary>
    public class JintegralPropagator2D: IPropagator
    {
        private readonly double magnificationOfJintegralRadius;
        private readonly IBulkIntegration jIntegrationRule;
        private readonly HomogeneousFractureMaterialField2D material;
        private readonly IAuxiliaryStates auxiliaryStatesStrategy;
        private readonly ISifCalculator sifCalculationStrategy;
        private readonly ICrackGrowthDirectionCriterion growthDirectionCriterion;
        private readonly ICrackGrowthLengthCriterion growthLengthCriterion;

        public PropagationLogger Logger { get; }

        /// <summary>
        /// Initializes a new instance of <see cref="JintegralPropagator2D"/>.
        /// </summary>
        /// <param name="magnificationOfJintegralRadius">The outer countour of the J-integral domain is defined as:
        ///     radius = magnification * sqrt(areaOfElementContainingTip). This parameter is the magnification. 
        ///     It should be at least 1.5 (see "Modeling quasi-static crack growth with the extended finite element 
        ///     method Part II: Numerical applications, Huang et al, 2003" page 7546). Usually values 2-3 are selected 
        ///     (see Ahmed thesis, 2009).</param>
        public JintegralPropagator2D(double magnificationOfJintegralRadius, IBulkIntegration jIntegrationRule,
            HomogeneousFractureMaterialField2D material,
            ICrackGrowthDirectionCriterion growthDirectionCriterion, ICrackGrowthLengthCriterion growthLengthCriterion)
        {
            this.magnificationOfJintegralRadius = magnificationOfJintegralRadius;
            this.jIntegrationRule = jIntegrationRule;
            this.material = material;
            this.auxiliaryStatesStrategy = new HomogeneousMaterialAuxiliaryStates(material);
            this.sifCalculationStrategy = new HomogeneousSifCalculator(material);
            this.growthDirectionCriterion = growthDirectionCriterion;
            this.growthLengthCriterion = growthLengthCriterion;
            this.Logger = new PropagationLogger();
        }

        public (double growthAngle, double growthLength) Propagate(Dictionary<int, Vector> subdomainFreeDisplacements, 
            double[] crackTipGlobal, TipCoordinateSystem tipSystem, IEnumerable<IXCrackElement> tipElements)
        {
            // TODO: Also check if the sifs do not violate the material toughness
            double[] sifs = ComputeSIFS(subdomainFreeDisplacements, crackTipGlobal, tipSystem, tipElements);
            double growthAngle = growthDirectionCriterion.ComputeGrowthAngle(sifs);
            double growthLength = growthLengthCriterion.ComputeGrowthLength(sifs);
            Logger.GrowthAngles.Add(growthAngle);
            Logger.GrowthLengths.Add(growthLength);
            return (growthAngle, growthLength);

        }

        private double[] ComputeSIFS(Dictionary<int, Vector> totalFreeDisplacements,
            double[] crackTip, TipCoordinateSystem tipSystem, IEnumerable<IXCrackElement> tipElements)
        {
            double interactionIntegralMode1 = 0.0, interactionIntegralMode2 = 0.0;
            IReadOnlyDictionary<IXCrackElement, double[]> elementWeights = 
                FindJintegralElementsAndNodalWeights(crackTip, tipElements);
            foreach (var pair in elementWeights)
            {
                IXCrackElement element = pair.Key;
                double[] nodalWeights = pair.Value;

                //TODO: This needs refactoring ASAP.
                XSubdomain subdomain = element.Subdomain;
                double[] elementDisplacements = 
                    subdomain.CalculateElementDisplacements(element, totalFreeDisplacements[subdomain.ID]);

                (double mode1I, double mode2I) = ComputeInteractionIntegrals(
                    element, Vector.CreateFromArray(elementDisplacements), nodalWeights, tipSystem);
                interactionIntegralMode1 += mode1I;
                interactionIntegralMode2 += mode2I;
            }

            double sifMode1 = sifCalculationStrategy.CalculateSif(interactionIntegralMode1);
            double sifMode2 = sifCalculationStrategy.CalculateSif(interactionIntegralMode2);

            Logger.SIFsMode1.Add(sifMode1);
            Logger.SIFsMode2.Add(sifMode2);

            return new double[] { sifMode1, sifMode2 };
        }

        private IReadOnlyDictionary<IXCrackElement, double[]> FindJintegralElementsAndNodalWeights(
            double[] crackTip, IEnumerable<IXCrackElement> tipElements)
        {
            Circle2D outerContour = 
                new Circle2D(crackTip[0], crackTip[1], ComputeRadiusOfJintegralOuterContour(tipElements));
            HashSet<IXCrackElement> intersectedElements = 
                MeshUtilities.FindElementsIntersectedByCircle(outerContour, tipElements.First());

            var elementsAndWeights = new Dictionary<IXCrackElement, double[]>();
            foreach (var element in intersectedElements)
            {
                // The relative position of the circle and the nodes was already calculated when checking the
                // circle-element intersection, but that method should be decoupled from assigning the nodal 
                // weights, even at the cost of some duplicate operations. What could be done more efficiently is 
                // caching the nodes and weights already processed by previous elements, but even then the cost of
                // processing each node will be increased by the lookup.
                double[] nodalWeights = new double[element.Nodes.Count];
                for (int n = 0; n < element.Nodes.Count; ++n)
                {
                    double signedDistance = outerContour.SignedDistanceOf(element.Nodes[n].Coordinates);
                    if (signedDistance > 0)
                    {
                        nodalWeights[n] = 0.0;
                    }
                    else // Node lies inside or exactly on the circle
                    {
                        nodalWeights[n] = 1.0;
                    }
                }
                elementsAndWeights.Add(element, nodalWeights);
            }
            return elementsAndWeights;
        }

        
        private double ComputeRadiusOfJintegralOuterContour(IEnumerable<IXCrackElement> tipElements)
        {
            // TODO: This method should directly return the elements and take care of cases near the domain boundaries (see Ahmed)
            // TODO: The J-integral radius should not exceed the last crack segment's length

            double maxTipElementArea = -1.0;
            foreach (var element in tipElements)
            {
                double elementArea = element.CalcBulkSizeCartesian();
                if (elementArea > maxTipElementArea) maxTipElementArea = elementArea;
            }
            return magnificationOfJintegralRadius * Math.Sqrt(maxTipElementArea);
        }

        private (double mode1I, double mode2I) ComputeInteractionIntegrals(IXCrackElement element, Vector nodalDisplacements,
            double[] nodalWeights, TipCoordinateSystem tipSystem)
        {
            double integralMode1 = 0.0;
            double integralMode2 = 0.0;
            foreach (GaussPoint gp in jIntegrationRule.GenerateIntegrationPoints(element))
            {
                // Nomenclature: global = global cartesian system, natural = element natural system, 
                // local = tip local cartesian system  

                // Coordinates of this gauss point in various systems
                EvalInterpolation evalInterpolation =
                    element.Interpolation.EvaluateAllAt(element.Nodes, gp.Coordinates);
                var point = new XPoint(2);
                point.Element = element;
                point.Coordinates[CoordinateSystem.ElementNatural] = gp.Coordinates;
                point.ShapeFunctions = evalInterpolation.ShapeFunctions;
                point.ShapeFunctionDerivatives = evalInterpolation.ShapeGradientsCartesian;
                double[] cartesianCoords = point.MapCoordinates(point.ShapeFunctions, point.Element.Nodes);
                point.Coordinates[CoordinateSystem.GlobalCartesian] = cartesianCoords;
                double[] polarCoords = tipSystem.MapPointGlobalCartesianToLocalPolar(cartesianCoords);
                TipJacobians tipJacobians = tipSystem.CalcJacobiansAt(polarCoords);

                // Material properties
                IMatrixView constitutive = material.FindMaterialAt(point).ConstitutiveMatrix;

                // State 1
                Matrix globalDisplacementGradState1 = element.CalcDisplacementFieldGradient(point, nodalDisplacements);
                Tensor2D globalStressState1 = CalcStressTensor(globalDisplacementGradState1, constitutive);
                Matrix localDisplacementGradState1 = tipSystem.
                    TransformVectorFieldDerivativesGlobalCartesianToLocalCartesian(globalDisplacementGradState1);
                Tensor2D localStressTensorState1 = tipSystem.
                    TransformTensorGlobalCartesianToLocalCartesian(globalStressState1);

                // Weight Function
                // TODO: There should be a method InterpolateScalarGradient(double[] nodalValues) in EvaluatedInterpolation
                // TODO: Rewrite this as a shapeGradients (matrix) * nodalWeights (vector) operation.
                var globalWeightGradient = Vector.CreateZero(2);
                for (int nodeIdx = 0; nodeIdx < element.Nodes.Count; ++nodeIdx)
                {
                    globalWeightGradient.AxpyIntoThis(
                        evalInterpolation.ShapeGradientsCartesian.GetRow(nodeIdx),
                        nodalWeights[nodeIdx]);
                }
                Vector localWeightGradient = 
                    tipSystem.TransformScalarFieldDerivativesGlobalCartesianToLocalCartesian(globalWeightGradient);

                // State 2
                // TODO: XCrackElement shouldn't have to pass tipCoordinate system to auxiliaryStates. 
                // It would be better to have CrackTip handle this and the coordinate transformations. That would also 
                // obey LoD, but a lot of wrapper methods would be required.
                AuxiliaryStatesTensors auxiliary = auxiliaryStatesStrategy.ComputeTensorsAt(polarCoords, tipJacobians);

                // Interaction integrals
                double integrandMode1 = ComputeJIntegrand(localWeightGradient, localDisplacementGradState1,
                    localStressTensorState1, auxiliary.DisplacementGradientMode1,
                    auxiliary.StrainTensorMode1, auxiliary.StressTensorMode1);
                double integrandMode2 = ComputeJIntegrand(localWeightGradient, localDisplacementGradState1,
                    localStressTensorState1, auxiliary.DisplacementGradientMode2,
                    auxiliary.StrainTensorMode2, auxiliary.StressTensorMode2);

                integralMode1 += integrandMode1 * evalInterpolation.Jacobian.DirectDeterminant * gp.Weight;
                integralMode2 += integrandMode2 * evalInterpolation.Jacobian.DirectDeterminant * gp.Weight;
            }
            return (integralMode1, integralMode2);
        }

        // In a non linear problem I would also have to pass the new displacements or I would have to update the
        // material state elsewhere.
        private static Tensor2D CalcStressTensor(Matrix displacementFieldGradient, IMatrixView constitutive)
        {
            double strainXX = displacementFieldGradient[0, 0];
            double strainYY = displacementFieldGradient[1, 1];
            double strainXYtimes2 = displacementFieldGradient[0, 1] + displacementFieldGradient[1, 0];

            // Should constitutive also be a tensor? Or  should I use matrices and vectors instead of tensors?
            double stressXX = constitutive[0, 0] * strainXX + constitutive[0, 1] * strainYY;
            double stressYY = constitutive[1, 0] * strainXX + constitutive[1, 1] * strainYY;
            double stressXY = constitutive[2, 2] * strainXYtimes2;

            return new Tensor2D(stressXX, stressYY, stressXY);
        }

        private static double ComputeJIntegrand(Vector weightGrad, Matrix displGrad1, Tensor2D stress1,
            Matrix displGrad2, Tensor2D strain2, Tensor2D stress2)
        {
            // Unrolled to greatly reduce mistakes. Alternatively Einstein notation products could be implementated
            // in Tensor2D (like the tensor-tensor multiplication is), but still some parts would have to be unrolled.
            // Perhaps vector (and scalar) gradients should also be accessed by component and derivative variable.

            double strainEnergy = stress1.MultiplyColon(strain2);
            double parenthesis0 = stress1.XX * displGrad2[0, 0] + stress1.XY * displGrad2[1, 0]
                + stress2.XX * displGrad1[0, 0] + stress2.XY * displGrad1[1, 0] - strainEnergy;
            double parenthesis1 = stress1.XY * displGrad2[0, 0] + stress1.YY * displGrad2[1, 0]
                + stress2.XY * displGrad1[0, 0] + stress2.YY * displGrad1[1, 0];
            return parenthesis0 * weightGrad[0] + parenthesis1 * weightGrad[1];
        }
    }
}
