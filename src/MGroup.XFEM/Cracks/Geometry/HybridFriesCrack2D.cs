using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Cracks.Geometry
{
	public class HybridFriesCrack2D : ICrack
	{
		private readonly XModel<IXCrackElement> model;
		private readonly IPropagator propagator;

		public HybridFriesCrack2D(XModel<IXCrackElement> model, CrackCurve2D crackGeometry, IPropagator propagator)
		{
			this.model = model;
			this.CrackCurve = crackGeometry;
			this.propagator = propagator;
		}

		public CrackCurve2D CrackCurve { get; }

		public List<ICrackObserver> Observers { get; } = new List<ICrackObserver>();

		public HashSet<IXCrackElement> ConformingElements { get; } = new HashSet<IXCrackElement>();

		public EnrichmentItem CrackBodyEnrichment { get; private set; }

		public IXGeometryDescription CrackGeometry => null;

		public EnrichmentItem CrackTipEnrichments { get; private set; }

		public int Dimension => 2;

		public HashSet<IXCrackElement> IntersectedElements { get; } = new HashSet<IXCrackElement>();

		public double[] TipCoordinates => null;

		public HashSet<IXCrackElement> TipElements { get; } = new HashSet<IXCrackElement>();

		public TipCoordinateSystem TipSystem => null;

		public int ID => CrackCurve.ID;

		public void CheckPropagation(IPropagationTermination termination)
		{
		}

		public IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments)
		{
			var stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };

			int enrichmentID = numCurrentEnrichments;

			// Crack body enrichment
			var stepEnrichmentFunc = new MockStepEnrichment();
			var stepEnrichedDofs = new IDofType[Dimension];
			for (int d = 0; d < Dimension; ++d)
			{
				stepEnrichedDofs[d] = new EnrichedDof(stepEnrichmentFunc, stdDofs[d]);
			}
			this.CrackBodyEnrichment = new EnrichmentItem(
				enrichmentID++, new IEnrichmentFunction[] { stepEnrichmentFunc }, stepEnrichedDofs);

			// Crack tip enrichments
			//TODO: For problems other than LEFM, use Abstract Factory pattern for tip enrichments, materials, propagators, etc.
			//var tipEnrichment = new IsotropicBrittleTipEnrichments2D(() => null/*lsmGeometry.TipSystem*/);
			//ICrackTipEnrichment[] tipEnrichmentFuncs = tipEnrichment.Functions;
			ICrackTipEnrichment[] tipEnrichmentFuncs = { new MockTipEnrichment(0), new MockTipEnrichment(1), new MockTipEnrichment(2), new MockTipEnrichment(2) };
			var tipEnrichedDofs = new List<IDofType>(4 * Dimension);
			for (int i = 0; i < tipEnrichmentFuncs.Length; ++i)
			{
				for (int d = 0; d < Dimension; ++d)
				{
					tipEnrichedDofs.Add(new EnrichedDof(tipEnrichmentFuncs[i], stdDofs[d]));
				}
			}
			this.CrackTipEnrichments = new EnrichmentItem(
				enrichmentID++, tipEnrichmentFuncs, tipEnrichedDofs.ToArray());

			return new EnrichmentItem[] { this.CrackBodyEnrichment, this.CrackTipEnrichments };
			//return new EnrichmentItem[0];
		}

		public HashSet<XNode> FindNodesNearFront(double maxDistance) 
			=> CrackCurve.FindNodesNearFront(model.Nodes.Values, maxDistance);

		public void InitializeGeometry()
		{
			CrackCurve.InitializeGeometry(model.Nodes.Values);
		}

		public void InteractWithMesh()
		{
			TipElements.Clear();

			foreach (IXCrackElement element in model.Elements.Values)
			{
				IElementDiscontinuityInteraction interaction = CrackCurve.Intersect(element);
				if (interaction.BoundaryOfGeometryInteractsWithElement)
				{
					TipElements.Add(element);
					element.RegisterInteractionWithCrack(this, interaction);
				}
				else if (interaction.RelativePosition == RelativePositionCurveElement.Intersecting)
				{
					IntersectedElements.Add(element);
					element.RegisterInteractionWithCrack(this, interaction);
				}
				else if (interaction.RelativePosition == RelativePositionCurveElement.Conforming)
				{
					ConformingElements.Add(element);
					element.RegisterInteractionWithCrack(this, interaction);
				}
			}

			// Call observers to pull any state they want
			foreach (ICrackObserver observer in Observers) observer.Update();
		}

		public void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
		{
			int numTips = CrackCurve.CrackFront.Vertices.Count;
			var frontGrowth = new CrackFrontPropagation();
			frontGrowth.AnglesAtTips = new double[numTips];
			frontGrowth.LengthsAtTips = new double[numTips];
			for (int i = 0; i < numTips; ++i)
			{
				double[] tipCoords = CrackCurve.CrackFront.Vertices[i].CoordsGlobal;
				double[] extensionVector = CrackCurve.CrackFront.CoordinateSystems[i].Tangent;
				(double growthAngle, double growthLength) = propagator.Propagate(
					algebraicModel, totalDisplacements, tipCoords, extensionVector, TipElements);
				frontGrowth.AnglesAtTips[i] = growthAngle;
				frontGrowth.LengthsAtTips[i] = growthLength;
			}
			CrackCurve.PropagateCrack(model.Nodes.Values, frontGrowth);
			CrackCurve.CheckAnglesBetweenCells();
		}
	}
}
