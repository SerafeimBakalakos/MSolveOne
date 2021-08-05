using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Entities
{
	public class XNode : INode
	{
		public XNode(int id, params double[] coordinates)
		{
			this.ID = id;
			this.Coordinates = coordinates;
		}

		public List<Constraint> Constraints { get; } = new List<Constraint>();


		public double[] Coordinates { get; }

		public new Dictionary<int, IXFiniteElement> ElementsDictionary { get; } = new Dictionary<int, IXFiniteElement>();

		//TODO: Perhaps this should be a Dictionary<EnrichmentItem, double[]> instead of storing them in EnrichmentFuncs
		public HashSet<EnrichmentItem> Enrichments { get; } = new HashSet<EnrichmentItem>();

		public Dictionary<IEnrichmentFunction, double> EnrichmentFuncs { get; } = new Dictionary<IEnrichmentFunction, double>();

		public int ID { get; }

		public bool IsEnriched => EnrichmentFuncs.Count > 0;

		public HashSet<int> Subdomains { get; } = new HashSet<int>();

		/// <summary>
		/// The id of the material phases this node belongs to. For multiphase problems, nodes may belong to an enclosed phase
		/// or the encompassing material, which is represented as the "default" phase. It is also possible that the enclosed 
		/// phases make up the whole domain, in which case there is no default phase. For problems without multiple material 
		/// phases, there is only one phase, the default onde.
		/// </summary>
		public int PhaseID { get; set; } = DefaultPhase.defaultPhaseID;

		public double X => Coordinates[0];

		public double Y => Coordinates[1];

		public double Z => Coordinates[2];

		public double CalculateDistanceFrom(XNode other)
		{
			if (Coordinates.Length == 2)
			{
				return Geometry.Utilities.Distance2D(this.Coordinates, other.Coordinates);
			}
			else if (Coordinates.Length == 3)
			{
				return Geometry.Utilities.Distance3D(this.Coordinates, other.Coordinates);
			}
			else throw new NotImplementedException();
		}
		public int CompareTo(INode other) => this.ID - other.ID;

		public override int GetHashCode() => ID.GetHashCode();
	}
}
