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
	public class HybridFriesCrack3D : ICrack
	{
		private readonly XModel<IXCrackElement> model;
		private readonly IPropagator propagator;

		public HybridFriesCrack3D(XModel<IXCrackElement> model, CrackSurface3D crackGeometry, IPropagator propagator)
		{
			this.model = model;
			this.CrackSurface = crackGeometry;
			this.propagator = propagator;
		}

		public CrackSurface3D CrackSurface { get; }

		public List<ICrackObserver> Observers { get; } = new List<ICrackObserver>();

		public HashSet<IXCrackElement> ConformingElements => null;

		public EnrichmentItem CrackBodyEnrichment => null;

		public IXGeometryDescription CrackGeometry => null;

		public EnrichmentItem CrackTipEnrichments => null;

		public int Dimension => 3;

		public HashSet<IXCrackElement> IntersectedElements => null;

		public double[] TipCoordinates => null;

		public HashSet<IXCrackElement> TipElements => null;

		public TipCoordinateSystem TipSystem => null;

		public int ID => CrackSurface.ID;

		public void CheckPropagation(IPropagationTermination termination)
		{
		}

		public IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments)
		{
			return new EnrichmentItem[0];
		}

		public void InitializeGeometry()
		{
			CrackSurface.InitializeGeometry(model.Nodes.Values);
		}

		public void InteractWithMesh()
		{
			// Call observers to pull any state they want
			foreach (ICrackObserver observer in Observers) observer.Update();
		}

		public void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
		{
			int numTips = CrackSurface.CrackFront.Vertices.Count;
			var frontGrowth = new CrackFrontPropagation();
			frontGrowth.AnglesAtTips = new double[numTips];
			frontGrowth.LengthsAtTips = new double[numTips];
			for (int i = 0; i < numTips; ++i)
			{
				double[] tipCoords = CrackSurface.CrackFront.Vertices[i].CoordsGlobal;
				double[] extensionVector = CrackSurface.CrackFront.CoordinateSystems[i].Extension;
				(double growthAngle, double growthLength) = propagator.Propagate(
					algebraicModel, totalDisplacements, tipCoords, extensionVector, TipElements);
				frontGrowth.AnglesAtTips[i] = growthAngle;
				frontGrowth.LengthsAtTips[i] = growthLength;
			}
			CrackSurface.PropagateCrack(model.Nodes.Values, frontGrowth);
			CrackSurface.CheckAnglesBetweenCells();
		}
	}
}
