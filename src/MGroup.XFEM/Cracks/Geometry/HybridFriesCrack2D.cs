using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.HybridFries;

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

		public HashSet<IXCrackElement> ConformingElements => null;

		public EnrichmentItem CrackBodyEnrichment => null;

		public IXGeometryDescription CrackGeometry => null;

		public EnrichmentItem CrackTipEnrichments => null;

		public int Dimension => 2;

		public HashSet<IXCrackElement> IntersectedElements => null;

		public double[] TipCoordinates => null;

		public HashSet<IXCrackElement> TipElements => null;

		public TipCoordinateSystem TipSystem => null;

		public int ID => CrackCurve.ID;

		public void CheckPropagation(IPropagationTermination termination)
		{
		}

		public IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments)
		{
			return new EnrichmentItem[0];
		}

		public void InitializeGeometry()
		{
			CrackCurve.InitializeGeometry(model);
		}

		public void InteractWithMesh()
		{
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
			CrackCurve.PropagateCrack(model, frontGrowth);
			CrackCurve.CheckAnglesBetweenCells();
		}
	}
}
