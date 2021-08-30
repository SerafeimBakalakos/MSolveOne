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

		public HybridFriesCrack3D(XModel<IXCrackElement> model, CrackSurface3D crackGeometry)
		{
			this.model = model;
			this.CrackSurface = crackGeometry;
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
			CrackSurface.InitializeGeometry(model);
		}

		public void InteractWithMesh()
		{
			// Call observers to pull any state they want
			foreach (ICrackObserver observer in Observers) observer.Update();
		}

		public void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
		{
		}
	}
}
