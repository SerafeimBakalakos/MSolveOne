using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks.FriesPropagation;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Cracks.Geometry
{
	public class TempEdgeCrack2D : ICrack
	{
		public readonly CrackCurve2D hybridGeometry;
		public readonly TipCoordinateSystemImplicit hybridTipCoordinateSystem;
		public FriesPropagator friesPropagator;

		private readonly List<double[]> crackPath; //TODO: should this also be stored in the pure geometry class?
		private readonly PolyLine2D initialCrack;
		private readonly OpenLsmSingleTip2D lsmGeometry; //TODO: Abstract this
		private readonly XModel<IXCrackElement> model;
		private readonly IPropagatorOLD propagator;

		public TempEdgeCrack2D(int id, PolyLine2D initialCrack, CrackCurve2D hybridGeometry, 
			XModel<IXCrackElement> model, IPropagatorOLD propagator)
		{
			this.hybridGeometry = hybridGeometry;
			this.hybridTipCoordinateSystem = new TipCoordinateSystemImplicit(hybridGeometry);

			this.ID = id;
			this.initialCrack = initialCrack;
			this.model = model;

			// Geometry
			this.lsmGeometry = new OpenLsmSingleTip2D(id);
			this.crackPath = new List<double[]>();
			this.propagator = propagator;
		}

		public HashSet<IXCrackElement> ConformingElements { get; } = new HashSet<IXCrackElement>();

		public EnrichmentItem CrackBodyEnrichment { get; private set; }

		public IReadOnlyList<double[]> CrackPath => crackPath;

		public IXGeometryDescription CrackGeometry => lsmGeometry;

		public EnrichmentItem CrackTipEnrichments { get; private set; }

		public HashSet<IXCrackElement> IntersectedElements { get; } = new HashSet<IXCrackElement>();

		public int ID { get; }

		public ISingleTipLsmGeometry LsmGeometry => lsmGeometry;

		public List<ICrackObserver> Observers { get; } = new List<ICrackObserver>();

		public double[] TipCoordinates => lsmGeometry.Tip;

		public HashSet<IXCrackElement> TipElements { get; } = new HashSet<IXCrackElement>();

		public TipCoordinateSystemExplicit TipSystem => lsmGeometry.TipSystem;

		public void CheckPropagation(IPropagationTermination termination)
		{
			double[] sifs = { propagator.Logger.SIFsMode1.Last(), propagator.Logger.SIFsMode2.Last() }; //TODO: These should be accessible without the logger.
			termination.Update(sifs, TipCoordinates);
		}

		public IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments)
		{
			int Dimension = 2;
			var stdDofs = new IDofType[] { StructuralDof.TranslationX, StructuralDof.TranslationY };

			int enrichmentID = numCurrentEnrichments;

			// Crack body enrichment
			var stepEnrichmentFunc = new CrackStepEnrichment_v2(hybridGeometry);
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
				new IsotropicBrittleTipEnrichments_v2.Func0(hybridTipCoordinateSystem),
				new IsotropicBrittleTipEnrichments_v2.Func1(hybridTipCoordinateSystem),
				new IsotropicBrittleTipEnrichments_v2.Func2(hybridTipCoordinateSystem),
				new IsotropicBrittleTipEnrichments_v2.Func3(hybridTipCoordinateSystem)
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
		{
			return hybridGeometry.FindNodesNearFront(model.Nodes.Values, maxDistance); ;

			var circle = new Circle2D(TipCoordinates, maxDistance); //TODO: This needs adapting for 3D
			return MeshUtilities.FindNodesInsideCircle(circle, TipElements.First());
		}

		public override int GetHashCode() => ID.GetHashCode();

		public void InitializeGeometry()
		{
			hybridGeometry.InitializeGeometry(model.Nodes.Values);

			lsmGeometry.Initialize(model.Nodes.Values, initialCrack);
			foreach (var vertex in initialCrack.Vertices) crackPath.Add(vertex);
		}

		public void InteractWithMesh()
		{
			IXGeometryDescription geometry = hybridGeometry;
			//IXGeometryDescription geometry = lsmGeometry;

			//TODO: Optimization: Do not go over the intersecting elements that are already stored here. Even better,
			//      implement some sort of narrow band (here or in the geometry class), to avoid checking all the elements in 
			//      the model. This will blur the difference between this class and the geometry one even further. Perhaps this
			//      method belongs to the geometry class, but that would mean that the geometry class, also stores interacted 
			//      elements, which is this class's responsibility.

			// I do not clear the interaction of all elements with this crack. As the crack propagates, the interaction may 
			// change from intersected/conforming to tip, but it will never be removed.
			TipElements.Clear(); 
			foreach (IXCrackElement element in model.Elements.Values)
			{
				IElementDiscontinuityInteraction interaction = geometry.Intersect(element);
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
			friesPropagator.Propagate(algebraicModel, totalDisplacements, hybridGeometry.CrackFront.Vertices[1].CoordsGlobal,
				hybridGeometry.CrackFront.CoordinateSystems[1].Tangent, TipElements);

			(double growthAngle, double growthLength) = propagator.Propagate(
				algebraicModel, totalDisplacements, lsmGeometry.Tip, lsmGeometry.TipSystem, TipElements);
			lsmGeometry.Update(model.Nodes.Values, growthAngle, growthLength);
			crackPath.Add(lsmGeometry.Tip);

			ICrackFront2D crackFront = hybridGeometry.CrackFront;
			Debug.Assert(crackFront.ActiveTips.Count == 1 && crackFront.ActiveTips[0] == 1);
			var frontGrowth = new CrackFrontPropagation();
			frontGrowth.AnglesAtTips = new double[] { growthAngle };
			frontGrowth.LengthsAtTips = new double[] { growthLength };
			hybridGeometry.PropagateCrack(model.Nodes.Values, frontGrowth);
			hybridGeometry.CheckAnglesBetweenCells();
		}
	}
}
