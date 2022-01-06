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

namespace MGroup.XFEM.Cracks.Geometry
{
	public class HybridFriesCrack3D : ICrack
	{
		private readonly XModel<IXCrackElement> model;
		private readonly bool oppositeHeavisideFunc;
		private readonly IPropagator propagator;

		public HybridFriesCrack3D(XModel<IXCrackElement> model, CrackSurface3D crackGeometry, IPropagator propagator,
			bool oppositeHeavisideFunc = false)
		{
			this.model = model;
			this.CrackGeometry_v2 = crackGeometry;
			this.propagator = propagator;
			this.oppositeHeavisideFunc = oppositeHeavisideFunc;
			this.FrontCoordinateSystem = new FrontCoordinateSystemImplicit(crackGeometry);
		}


		public List<ICrackObserver> Observers { get; } = new List<ICrackObserver>();

		public HashSet<IXCrackElement> ConformingElements { get; } = new HashSet<IXCrackElement>();

		public EnrichmentItem CrackBodyEnrichment { get; private set; }

		public IXGeometryDescription CrackGeometry => CrackGeometry_v2;

		public CrackSurface3D CrackGeometry_v2 { get; }

		public EnrichmentItem CrackTipEnrichments { get; private set; }

		public int Dimension => 3;

		public HashSet<IXCrackElement> IntersectedElements { get; } = new HashSet<IXCrackElement>();

		public double[] TipCoordinates => null;

		public HashSet<IXCrackElement> TipElements { get; } = new HashSet<IXCrackElement>();

		public IFrontCoordinateSystem FrontCoordinateSystem { get; }
		public FrontCoordinateSystemExplicit FrontSystem => null;

		public int ID => CrackGeometry_v2.ID;

		public void CheckPropagation(IPropagationTermination termination)
		{
		}

		public IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments)
		{
			var stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ };

			int enrichmentID = numCurrentEnrichments;

			// Crack body enrichment
			var stepEnrichmentFunc = new CrackStepEnrichment_v2(CrackGeometry_v2, oppositeHeavisideFunc);
			var stepEnrichedDofs = new IDofType[Dimension];
			for (int d = 0; d < Dimension; ++d)
			{
				stepEnrichedDofs[d] = new EnrichedDof(stepEnrichmentFunc, stdDofs[d]);
			}
			this.CrackBodyEnrichment = new EnrichmentItem(
				enrichmentID++, new IEnrichmentFunction[] { stepEnrichmentFunc }, stepEnrichedDofs);

			// Crack tip enrichments
			//TODO: For problems other than LEFM, use Abstract Factory pattern for tip enrichments, materials, propagators, etc.
			ICrackTipEnrichment[] tipEnrichmentFuncs =
			{
				new IsotropicBrittleTipEnrichments_v2.Func0(FrontCoordinateSystem),
				new IsotropicBrittleTipEnrichments_v2.Func1(FrontCoordinateSystem),
				new IsotropicBrittleTipEnrichments_v2.Func2(FrontCoordinateSystem),
				new IsotropicBrittleTipEnrichments_v2.Func3(FrontCoordinateSystem)
			};
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
			=> CrackGeometry_v2.FindNodesNearFront(model.Nodes.Values, maxDistance);

		public void InitializeGeometry()
		{
			CrackGeometry_v2.InitializeGeometry(model.Nodes.Values);
		}

		public void InteractWithMesh()
		{
			TipElements.Clear();

			//foreach (IXCrackElement element in model.Elements.Values)
			//{
			//	IElementDiscontinuityInteraction interaction = CrackGeometry_v2.Intersect(element);
			//	if (interaction.BoundaryOfGeometryInteractsWithElement)
			//	{
			//		TipElements.Add(element);
			//		element.RegisterInteractionWithCrack(this, interaction);
			//	}
			//	else if (interaction.RelativePosition == RelativePositionCurveElement.Intersecting)
			//	{
			//		IntersectedElements.Add(element);
			//		element.RegisterInteractionWithCrack(this, interaction);
			//	}
			//	else if (interaction.RelativePosition == RelativePositionCurveElement.Conforming)
			//	{
			//		ConformingElements.Add(element);
			//		element.RegisterInteractionWithCrack(this, interaction);
			//	}
			//}

			// Call observers to pull any state they want
			foreach (ICrackObserver observer in Observers) observer.Update();
		}

		public void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
		{
			ICrackFront3D crackFront = CrackGeometry_v2.CrackFront;
			int numTips = crackFront.ActiveTips.Count;
			var tipSystems = new ICrackTipSystem[numTips];
			for (int i = 0; i < numTips; ++i)
			{
				int vertexIdx = crackFront.ActiveTips[i];
				tipSystems[i] = crackFront.CoordinateSystems[vertexIdx];
			}

			var frontGrowth = new CrackFrontPropagation();
			(frontGrowth.AnglesAtTips, frontGrowth.LengthsAtTips) =
				propagator.Propagate(algebraicModel, totalDisplacements, tipSystems);

			CrackGeometry_v2.PropagateCrack(model.Nodes.Values, frontGrowth);
			CrackGeometry_v2.CheckAnglesBetweenCells();
		}
	}
}
